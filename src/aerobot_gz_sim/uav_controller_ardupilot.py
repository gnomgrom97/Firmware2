import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import CommandBool, ParamSetV2, SetMode, CommandTOL
from rcl_interfaces.msg import ParameterType
from sensor_msgs.msg import LaserScan

# Ограничения движения и настройки локального обхода по данным лидара.
NAV_SPEED_M_S = 0.5  # Максимальная скорость навигации, м/с.
SLOW_NAV_SPEED_M_S = 0.25  # Сниженная скорость при проходе узких мест (проёмов).
OBSTACLE_STOP_DISTANCE_M = 1.25  # Заранее начинаем выбирать свободный проход.
OBSTACLE_SECTOR_HALF_ANGLE_RAD = math.radians(8.0)  # Узкий сектор: рамка проёма сбоку не должна считаться препятствием.
DETOUR_STEP_M = 1.0  # Длина waypoint в направлении выбранного просвета, м.
DETOUR_MIN_CLEARANCE_M = 1.0  # Минимальная дальность стены в проверяемом секторе, м.
DETOUR_ANGLE_STEP_RAD = math.radians(20.0)  # Шаг перебора углов для поиска прохода.10.0
OBSTACLE_MARGIN_M = 0.5  # Запас сверх длины короткой цели обхода, м.
MAX_DETOUR_STEPS = 10000000000  # Максимальное число обходных шагов за один вызов goto().

# --- Безопасность прохода узких мест (дверных проёмов) ---
DRONE_RADIUS_M = 0.6  # Физический радиус дрона с запасом (корпус + пропеллеры).
MIN_SAFE_CLEARANCE_M = max(0.9, DRONE_RADIUS_M * 2.2)  # Нижний порог требуемого зазора — не даём ему падать ниже этого значения у самой цели.
DOORWAY_CENTERING_RANGE_M = 1.0  # На этой дистанции от цели включаем центрирование между стенами проёма.
CENTERING_SIDE_MIN_ANGLE_RAD = math.radians(1.0)  # Боковой сектор начинается здесь (не считаем сам проём стеной).
CENTERING_SIDE_MAX_ANGLE_RAD = math.radians(360.0)  # ...и заканчивается здесь.
CENTERING_GAIN = 0.5  # Доля разницы (право - лево) зазоров, на которую сдвигаем цель вбок.
CENTERING_MAX_OFFSET_M = 0.6  # Ограничение максимального бокового сдвига за один шаг.


class UavControllerArduPilot(Node):
    """
    Контроллер для ArduPilot (режим GUIDED).
    Ключевое отличие от PX4: взлёт делается ЯВНОЙ командой через сервис
    cmd/takeoff, а не потоком setpoint-ов. Поток setpoint-ов нужен
    уже ПОСЛЕ взлёта — для навигации по точкам.
    """

    def __init__(self, ns="uav1"):
        super().__init__("uav_controller_ap")
        self.ns = ns  # Пространство имён MAVROS, например uav1.
        self.state = State()  # Последнее состояние автопилота.
        self.pose = PoseStamped()  # Последняя локальная позиция и ориентация дрона.
        # Последний лидарный скан и время его получения нужны для проверки свежести.
        self.latest_scan = None  # Последнее сообщение LaserScan или None до первого скана.
        self.latest_scan_time = 0.0  # Время получения последнего скана по системным часам.

        # BEST_EFFORT подходит для потоковых данных позиции и лидара.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)  # QoS-профиль сенсорных потоков.

        # Эти подписки получают состояние полётного контроллера и текущую позицию.
        self.create_subscription(State, f"/{ns}/state", self._state_cb, 10)
        self.create_subscription(PoseStamped, f"/{ns}/local_position/pose",
                                  self._pose_cb, qos)
        self.last_statustexts = []  # Буфер последних текстовых сообщений от FCU.
        self.create_subscription(StatusText, f"/{ns}/statustext/recv",
                      self._statustext_cb, qos)
        # Лидар нужен goto() для обнаружения стен и выбора свободного направления.
        self.create_subscription(LaserScan, f"/{ns}/scan", self._scan_cb, qos)
        self.setpoint_pub = self.create_publisher(
            PoseStamped, f"/{ns}/setpoint_position/local", 10)
        self.vel_pub = self.create_publisher(
            TwistStamped, f"/{ns}/setpoint_velocity/cmd_vel", 10)

        self.arm_cli = self.create_client(CommandBool, f"/{ns}/cmd/arming")
        self.mode_cli = self.create_client(SetMode, f"/{ns}/set_mode")
        self.takeoff_cli = self.create_client(CommandTOL, f"/{ns}/cmd/takeoff")
        self.land_cli = self.create_client(CommandTOL, f"/{ns}/cmd/land")
        # Через MAVROS задаются скорость и ускорение навигации ArduPilot.
        self.param_set_cli = self.create_client(ParamSetV2, f"/{ns}/param/set")

        # Таймер постоянно отправляет FCU текущую цель позиции или скорости.
        self._sp_target = None  # Целевая позиция (x, y, z, yaw), если включён режим позиции.
        self._vel_target = None  # Целевая скорость (vx, vy, vz, yaw_rate), если включён режим скорости.
        self._sp_timer = self.create_timer(0.05, self._sp_timer_cb)  # Таймер публикации setpoint с частотой 20 Гц.

    def _state_cb(self, msg):
        # Сохраняем состояние, например режим полёта и armed.
        self.state = msg  # msg — новое сообщение State от FCU.

    def _pose_cb(self, msg):
        # Текущая локальная позиция нужна для проверки достижения целей.
        self.pose = msg  # msg — текущая поза в локальной системе координат.

    def _scan_cb(self, msg):
        # Запоминаем скан и время его получения для контроля устаревших данных.
        self.latest_scan = msg  # msg — новый полный круговой скан лидара.
        self.latest_scan_time = time.time()  # Время прихода нужно для поиска зависшего потока.

    def _statustext_cb(self, msg):
        # Храним только последние сообщения, чтобы показывать причины отказа FCU.
        text = msg.text  # Текстовое предупреждение или состояние от FCU.
        self.last_statustexts.append(text)
        if len(self.last_statustexts) > 20:
            self.last_statustexts.pop(0)
        low = text.lower()  # Нижний регистр позволяет искать ключевые слова без учёта регистра.
        if "prearm" in low or "pre-arm" in low or "arm" in low:
            self.get_logger().info(f"FCU statustext: {text}")

    def wait_for_prearm_good(self, timeout=180.0, poll_sec=1.0):
        """
        Ждёт сообщения 'PreArm: ...' пропадания ошибок / появления 'good' в
        последних статустекстах. Если ArduPilot не шлёт явное 'pre-arm good' в
        этот топик (бывает по-разному в разных версиях), просто ждём заданное
        время и параллельно логируем всё, что реально приходит, чтобы было видно.
        """
        self.get_logger().info(
            "Waiting for FCU pre-arm checks to pass (watching statustext)...")
        # Ожидание обрабатывает ROS-сообщения, чтобы обновлялись callbacks.
        t0 = time.time()  # Момент начала ожидания pre-arm.
        last_count = 0  # Число сообщений в буфере при предыдущем проходе цикла.
        last_change_time = time.time()  # Время последнего нового сообщения FCU.
        QUIET_PERIOD = 3.0  # Пауза без сообщений, после которой поток считают тихим.

        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)

            if self.last_statustexts:
                latest = self.last_statustexts[-1].lower()  # Последнее сообщение FCU для проверки готовности.
                if "pre-arm good" in latest or "prearm good" in latest:
                    self.get_logger().info("Pre-arm good detected.")
                    return True

            if len(self.last_statustexts) != last_count:
                last_count = len(self.last_statustexts)  # Обновляем размер буфера сообщений.
                last_change_time = time.time()  # Запоминаем время появления нового сообщения.
            elif time.time() - last_change_time > QUIET_PERIOD and time.time() - t0 > 10.0:
                # поток сообщений успокоился хотя бы через 10с после старта — пробуем
                self.get_logger().info(
                    "Statustext stream quiet, assuming FCU is ready to attempt arm.")
                return True

        self.get_logger().warn(
            "Timed out waiting for pre-arm signal; will attempt arm anyway.")
        return False

    def _sp_timer_cb(self):
        # Скоростной режим имеет приоритет, если задан (используется в Задании 3)
        if self._vel_target is not None:
            vx, vy, vz, yaw_rate = self._vel_target  # Скорости по осям и скорость поворота.
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = vx
            msg.twist.linear.y = vy
            msg.twist.linear.z = vz
            msg.twist.angular.z = yaw_rate
            self.vel_pub.publish(msg)
        elif self._sp_target is not None:
            # Формируем PoseStamped для ArduPilot из текущей позиционной цели.
            x, y, z, yaw = self._sp_target  # Целевая позиция и курс для публикации.
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.orientation.w = math.cos(yaw / 2.0)
            self.setpoint_pub.publish(msg)

    def set_setpoint_target(self, x, y, z, yaw=0.0):
        # Переключаемся на позиционное управление и нормализуем типы ROS-полей.
        self._vel_target = None
        self._sp_target = (float(x), float(y), float(z), float(yaw))

    def set_velocity_target(self, vx, vy, vz, yaw_rate=0.0):
        """Задаёт линейные скорости по X/Y/Z и скорость поворота вокруг Z."""
        # При задании скорости отключаем позиционный setpoint.
        self._sp_target = None
        self._vel_target = (
            float(vx), float(vy), float(vz), float(yaw_rate)
        )

    def stop(self):
        self.set_velocity_target(0.0, 0.0, 0.0, 0.0)

    # ---------- вспомогательные ----------

    def spin_for(self, seconds):
        # Не блокируем обработку ROS callbacks во время ожидания.
        t0 = time.time()  # Начало интервала ожидания.
        while rclpy.ok() and time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def spin_until(self, cond, timeout=15.0):
        # Обрабатываем сообщения, пока условие не выполнится или не истечёт таймаут.
        t0 = time.time()  # Начало ожидания условия cond.
        while rclpy.ok() and not cond() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return cond()

    # ---------- команды FCU ----------

    def set_mode(self, mode, timeout=10.0):
        # Do not send a mode request until MAVROS reports an FCU connection.
        if not self.spin_until(lambda: self.state.connected, timeout=timeout):
            self.get_logger().error(
                f"Cannot set mode {mode}: FCU is not connected "
                f"(current mode: {self.state.mode or 'unknown'})."
            )
            return False

        # Сначала отправляем запрос MAVROS, затем подтверждаем смену по /state.
        while not self.mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set_mode service...")
        req = SetMode.Request()  # Запрос MAVROS на смену режима.
        req.custom_mode = mode  # Требуемый режим, например GUIDED или LAND.
        future = self.mode_cli.call_async(req)  # Асинхронный ответ сервиса.
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            self.get_logger().error(
                f"set_mode({mode}) timed out waiting for MAVROS response "
                f"(FCU connected: {self.state.connected}, current mode: "
                f"{self.state.mode or 'unknown'})."
            )
            return False

        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"set_mode({mode}) service call raised: {exc}")
            return False

        if result is None:
            self.get_logger().error(f"set_mode({mode}) returned no response.")
            return False
        if not result.mode_sent:
            recent_status = "; ".join(self.last_statustexts[-3:]) or "none received"
            self.get_logger().error(
                f"MAVROS rejected set_mode({mode}) (FCU connected: "
                f"{self.state.connected}, current mode: "
                f"{self.state.mode or 'unknown'}, recent FCU status: "
                f"{recent_status})."
            )
            return False
        ok = self.spin_until(lambda: self.state.mode == mode, timeout=timeout)  # Подтверждение режима через State.
        if not ok:
            recent_status = "; ".join(self.last_statustexts[-3:]) or "none received"
            self.get_logger().error(
                f"set_mode({mode}) was sent but FCU did not switch "
                f"(current mode: {self.state.mode or 'unknown'}, "
                f"recent FCU status: {recent_status})."
            )
        return ok

    def set_navigation_speed(self, speed_m_s=NAV_SPEED_M_S, timeout=5.0):
        """Ограничивает скорость и ускорение ArduPilot (значения в см-системе)."""
        if not self.param_set_cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("MAVROS param/set service is unavailable.")
            return False

        # Параметры WPNAV_SPEED и WPNAV_ACCEL в ArduPilot задаются в сантиметрах.
        parameters = (  # Параметры автопилота и значения скорости/ускорения в см/с.
            ("WPNAV_SPEED", speed_m_s * 100.0),
            ("WPNAV_ACCEL", speed_m_s * 100.0),
        )
        for name, value in parameters:  # Поочерёдно отправляем каждый параметр.
            request = ParamSetV2.Request()  # Новый запрос установки параметра.
            request.force_set = False
            request.param_id = name
            request.value.type = ParameterType.PARAMETER_DOUBLE
            request.value.double_value = value
            future = self.param_set_cli.call_async(request)  # Будущий ответ MAVROS.
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            result = future.result()  # Ответ с признаком успешной установки.
            if result is None or not result.success:
                self.get_logger().error(f"Не удалось задать параметр {name}.")
                return False

        self.get_logger().info(
            f"Скорость ограничена до {speed_m_s:.1f} м/с, ускорение плавное."
        )
        return True

    def _yaw_from_pose(self):
        # Преобразуем quaternion ориентации в угол курса вокруг оси Z.
        orientation = self.pose.pose.orientation  # Quaternion текущей ориентации дрона.
        return math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )

    def _obstacle_distance_toward(self, target_x, target_y):
        """Возвращает ближайшую дальность лидара в секторе к целевой точке."""
        # Без свежего скана безопасно оценить путь нельзя.
        if self.latest_scan is None or time.time() - self.latest_scan_time > 2.0:
            return None

        target_bearing = self._bearing_to_scan_angle(target_x, target_y)  # Направление на цель относительно лидара.
        nearest = float("inf")  # Ближайшая дальность в секторе; inf означает отсутствие отражений.
        found_beam = False  # Проверяем, покрыт ли путь лучами лидара.
        scan = self.latest_scan  # Локальная ссылка на сообщение скана.
        # Угол каждого луча вычисляется из angle_min и angle_increment сообщения.
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance < scan.range_min:
                continue
            beam_angle = scan.angle_min + index * scan.angle_increment
            difference = math.atan2(
                math.sin(beam_angle - target_bearing),
                math.cos(beam_angle - target_bearing),
            )
            if abs(difference) <= OBSTACLE_SECTOR_HALF_ANGLE_RAD:
                found_beam = True
                nearest = min(nearest, distance)
        # Непроверенное направление нельзя считать свободным.
        if not found_beam:
            return 0.0
        # Вычитаем радиус дрона: лидар меряет расстояние до стены от точки
        # установки, а не до ближайшей части корпуса/пропеллеров дрона.
        return max(0.0, nearest - DRONE_RADIUS_M)

    def _bearing_to_scan_angle(self, target_x, target_y):
        # Переводим направление на цель из мировой системы в систему лидара.
        dx = target_x - self.pose.pose.position.x  # Смещение до цели по мировой оси X.
        dy = target_y - self.pose.pose.position.y  # Смещение до цели по мировой оси Y.
        return math.atan2(dy, dx) - self._yaw_from_pose()

    def _clearance_at_bearing(self, bearing):
        """Ищет свободное расстояние в узком секторе лидара."""
        scan = self.latest_scan  # Текущий скан для оценки выбранного направления.
        nearest = float("inf")  # Минимальная дальность в секторе.
        found_beam = False  # Был ли луч лидара в проверяемом секторе.
        for index, distance in enumerate(scan.ranges):
            beam_angle = scan.angle_min + index * scan.angle_increment
            difference = math.atan2(
                math.sin(beam_angle - bearing),
                math.cos(beam_angle - bearing),
            )
            if abs(difference) > OBSTACLE_SECTOR_HALF_ANGLE_RAD:
                continue
            found_beam = True
            if not math.isfinite(distance) or distance < scan.range_min:
                continue
            nearest = min(nearest, distance)
        # Если в скане нет лучей в этом секторе, считать направление свободным нельзя.
        if not found_beam:
            return 0.0
        if math.isinf(nearest):
            return scan.range_max
        return max(0.0, nearest - DRONE_RADIUS_M)

    def _side_clearances(self, bearing):
        """
        Измеряет ближайший зазор слева и справа от курса `bearing`, в поясе
        углов [CENTERING_SIDE_MIN, CENTERING_SIDE_MAX] по обе стороны.
        Используется, чтобы понять, насколько дрон смещён от середины
        прохода (дверного проёма), а не просто "есть ли препятствие впереди".
        """
        scan = self.latest_scan
        left = float("inf")
        right = float("inf")
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance < scan.range_min:
                continue
            beam_angle = scan.angle_min + index * scan.angle_increment
            rel = math.atan2(
                math.sin(beam_angle - bearing), math.cos(beam_angle - bearing)
            )
            # Правая сторона — положительный угол относительно курса.
            if CENTERING_SIDE_MIN_ANGLE_RAD < rel < CENTERING_SIDE_MAX_ANGLE_RAD:
                right = min(right, distance)
            # Левая сторона — отрицательный угол относительно курса.
            elif -CENTERING_SIDE_MAX_ANGLE_RAD < rel < -CENTERING_SIDE_MIN_ANGLE_RAD:
                left = min(left, distance)
        return left, right

    def _centering_offset(self, target_x, target_y):
        """
        Если цель близко (в пределах DOORWAY_CENTERING_RANGE_M) и видны стены
        по обе стороны курса — возвращает (dx, dy), боковой сдвиг цели к
        середине прохода. Если центрировать не от чего (нет стен с одной из
        сторон, или слишком далеко) — возвращает (0.0, 0.0).
        """
        if self.latest_scan is None:
            return 0.0, 0.0

        target_distance = math.dist(
            (self.pose.pose.position.x, self.pose.pose.position.y),
            (target_x, target_y),
        )
        if target_distance > DOORWAY_CENTERING_RANGE_M:
            return 0.0, 0.0

        bearing = self._bearing_to_scan_angle(target_x, target_y)
        left, right = self._side_clearances(bearing)
        if math.isinf(left) or math.isinf(right):
            # Видим стену только с одной стороны (или не видим вовсе) —
            # недостаточно данных, чтобы безопасно центрироваться.
            return 0.0, 0.0

        # Положительная ошибка значит "справа теснее" -> нужно сдвинуться влево.
        error = (right - left) / 2.0
        error = max(-CENTERING_MAX_OFFSET_M, min(CENTERING_MAX_OFFSET_M, error * CENTERING_GAIN))

        world_bearing = bearing + self._yaw_from_pose()
        # Перпендикуляр к курсу (поворот на +90°) задаёт направление "влево".
        perp = world_bearing + math.pi / 2.0
        dx = error * math.cos(perp)
        dy = error * math.sin(perp)
        return dx, dy

    def _choose_detour_target(self, goal_x, goal_y, goal_z):
        """Выбирает локальный свободный шаг; карту комнаты метод не строит."""
        target_bearing = self._bearing_to_scan_angle(goal_x, goal_y)  # Базовое направление к исходной цели.
        candidates = []  # Допустимые направления и отклонение от прямого курса.
        angle_steps = int(math.radians(150.0) / DETOUR_ANGLE_STEP_RAD)  # Число проверок в веере ±150°.
        # Сначала предпочитаем прямой курс; дальность просвета разрешает равные углы.
        for step in range(-angle_steps, angle_steps + 1):
            offset = step * DETOUR_ANGLE_STEP_RAD  # Отклонение от направления к цели.
            bearing = target_bearing + offset  # Проверяемый угол относительно лидара.
            clearance = self._clearance_at_bearing(bearing)  # Измеренный просвет по этому углу.
            if clearance < DETOUR_MIN_CLEARANCE_M:
                continue
            candidates.append((abs(offset), -clearance, bearing))

        if not candidates:
            return None

        _, _, best_bearing = min(candidates)  # Наименее отклонённый безопасный угол.
        world_bearing = best_bearing + self._yaw_from_pose()  # Перевод угла в мировую систему координат.
        position = self.pose.pose.position  # Текущая позиция для построения точки обхода.
        return (
            position.x + DETOUR_STEP_M * math.cos(world_bearing),
            position.y + DETOUR_STEP_M * math.sin(world_bearing),
            goal_z,
        )

    def _try_arm_once(self, timeout=10.0):
        # Одна попытка arm; состояние armed дополнительно подтверждаем по топику.
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for arming service...")
        req = CommandBool.Request()  # Запрос на включение моторов.
        req.value = True  # True означает команду армирования.
        future = self.arm_cli.call_async(req)  # Асинхронный ответ сервиса arm.
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None or not future.result().success:
            return False
        return self.spin_until(lambda: self.state.armed, timeout=timeout)

    def arm(self, max_attempts=10, retry_delay=3.0, per_attempt_timeout=10.0):
        """
        Пытается армировать несколько раз подряд (preflight-проверки ArduPilot
        могут пройти не сразу). После каждой неудачи печатает последние
        сообщения от FCU из statustext, чтобы была видна реальная причина.
        """
        # Повторяем arm: pre-arm проверки симулятора могут завершиться не сразу.
        for attempt in range(1, max_attempts + 1):  # Номер текущей попытки армирования.
            self.get_logger().info(f"Arm attempt {attempt}/{max_attempts}...")
            if self._try_arm_once(timeout=per_attempt_timeout):
                self.get_logger().info("Armed successfully.")
                return True

            if self.last_statustexts:
                self.get_logger().error(
                    "ARM REJECTED. Last FCU messages: " +
                    " | ".join(self.last_statustexts[-5:])
                )
            else:
                self.get_logger().error(
                    "ARM REJECTED. No statustext messages received yet — "
                    "FCU may still be initializing."
                )

            if attempt < max_attempts:
                self.spin_for(retry_delay)

        self.get_logger().error(f"Arming failed after {max_attempts} attempts.")
        return False

    def takeoff(self, altitude=2.0, timeout=15.0):
        """Явная команда взлёта — обязательна для ArduPilot."""
        while not self.takeoff_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for takeoff service...")
        # ArduPilot получает отдельный сервисный запрос на взлёт.
        req = CommandTOL.Request()  # Запрос MAVROS на команду взлёта.
        req.altitude = float(altitude)  # Требуемая высота взлёта в метрах.
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw = 0.0
        future = self.takeoff_cli.call_async(req)  # Асинхронный ответ FCU на команду взлёта.
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None or not future.result().success:
            self.get_logger().error("TAKEOFF command REJECTED by FCU")
            return False

        self.get_logger().info(f"Climbing to {altitude}m...")
        t0 = time.time()  # Начало ожидания набора высоты.
        climb_timeout = max(150.0, altitude * 8 + 10)  # Предельное время набора высоты.
        reached_altitude = False  # Станет True после подтверждения высоты по pose.
        # Ждём фактическую высоту по локальной позиции, а не только ответ сервиса.
        while rclpy.ok() and time.time() - t0 < climb_timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if abs(self.pose.pose.position.z - altitude) < 0.2:#self.pose #хранит положение: x y z
                self.get_logger().info("Reached target altitude.")
                reached_altitude = True
                break
        if not reached_altitude:
            self.get_logger().error(
                f"Timed out waiting for altitude {altitude}m "
                f"(current: {self.pose.pose.position.z:.2f}m)."
            )
            return False
        self.set_setpoint_target(
            self.pose.pose.position.x, self.pose.pose.position.y, altitude)#self.pose #хранит положение: x y z
        return True

    def run_mission(self, altitude=2.0):
        # Полная последовательность запуска: pre-arm, GUIDED, скорость, arm, взлёт.
        self.wait_for_prearm_good(timeout=280.0)

        self.get_logger().info("Switching to GUIDED...")
        if not self.set_mode("GUIDED"):
            return False

        if not self.set_navigation_speed():
            return False

        if not self.arm():
            return False
        return self.takeoff(altitude=altitude)

    def goto(self, x, y, z, yaw=0.0, hold_sec=0.0, tol=0.3, timeout=150.0):
        # Движется к цели, проверяя путь лидаром и при необходимости добавляя локальные шаги.
        goal = (float(x), float(y), float(z))  # Исходная конечная позиция движения.
        active_target = goal  # Текущая цель: конечная точка или временная точка обхода.
        detour_steps = 0  # Сколько обходных точек уже выбрано в этом вызове goto().
        slowed_down = False  # Снижали ли уже скорость при входе в зону центрирования.
        t0 = time.time()  # Начало общего таймаута полёта к цели.
        while rclpy.ok() and time.time() - t0 < timeout:
            # Боковое центрирование: если цель близко и видны стены по обе
            # стороны курса (типичная ситуация для дверного проёма), сдвигаем
            # фактически посылаемую точку к середине прохода, не меняя саму
            # цель active_target (чтобы не ломать логику обхода препятствий).
            offset_dx, offset_dy = self._centering_offset(active_target[0], active_target[1])
            if (offset_dx != 0.0 or offset_dy != 0.0) and not slowed_down:
                # Снижаем скорость один раз при входе в узкий проход — точность
                # важнее скорости, пока дрон не прошёл проём насквозь.
                self.set_navigation_speed(SLOW_NAV_SPEED_M_S)
                slowed_down = True
            send_target = (
                active_target[0] + offset_dx,
                active_target[1] + offset_dy,
                active_target[2],
            )
            self.set_setpoint_target(*send_target, yaw)
            rclpy.spin_once(self, timeout_sec=0.05)
            obstacle_distance = self._obstacle_distance_toward(  # Ближайшее препятствие на пути к активной цели.
                active_target[0], active_target[1]
            )
            # Устаревший лидарный скан означает, что продолжать движение небезопасно.
            if obstacle_distance is None:
                self.set_setpoint_target(
                    self.pose.pose.position.x,
                    self.pose.pose.position.y,
                    self.pose.pose.position.z,
                    yaw,
                )
                self.get_logger().error(
                    "Данные LiDAR отсутствуют или устарели. Останавливаю движение."
                )
                if slowed_down:
                    self.set_navigation_speed(NAV_SPEED_M_S)
                return False
            target_distance = math.dist(  # Горизонтальная дальность до текущей цели.
                (self.pose.pose.position.x, self.pose.pose.position.y),
                active_target[:2],
            )
            required_clearance = max(  # Не опускаем порог ниже безопасного минимума даже у самой цели.
                MIN_SAFE_CLEARANCE_M,
                min(OBSTACLE_STOP_DISTANCE_M, target_distance + OBSTACLE_MARGIN_M),
            )
            # Если путь закрыт, ищем боковой сектор; при неудаче возвращаем False.
            if obstacle_distance < required_clearance:
                if detour_steps >= MAX_DETOUR_STEPS:
                    self.set_setpoint_target(
                        self.pose.pose.position.x,
                        self.pose.pose.position.y,
                        self.pose.pose.position.z,
                        yaw,
                    )
                    self.get_logger().error("Лимит обходных шагов достигнут.")
                    if slowed_down:
                        self.set_navigation_speed(NAV_SPEED_M_S)
                    return False

                # Если уже обходим, ищем продолжение этого же курса, не прыгая назад к исходной цели.
                detour_target = self._choose_detour_target(*active_target)
                if detour_target is None:
                    self.set_setpoint_target(
                        self.pose.pose.position.x,
                        self.pose.pose.position.y,
                        self.pose.pose.position.z,
                        yaw,
                    )
                    self.get_logger().error(
                        f"Впереди препятствие на {obstacle_distance:.2f} м, "
                        "свободного направления для обхода не найдено."
                    )
                    if slowed_down:
                        self.set_navigation_speed(NAV_SPEED_M_S)
                    return False

                active_target = detour_target  # Направляем дрон к выбранной точке обхода.
                detour_steps += 1  # Учитываем шаг для ограничения бесконечного обхода.
                self.get_logger().warn(
                    f"Объезжаю препятствие: шаг {detour_steps}, "
                    f"просвет впереди {obstacle_distance:.2f} м."
                )
                continue

            dx = self.pose.pose.position.x - goal[0]  # Ошибка конечной позиции по X.
            dy = self.pose.pose.position.y - goal[1]  # Ошибка конечной позиции по Y.
            dz = self.pose.pose.position.z - goal[2]  # Ошибка конечной высоты по Z.
            if (dx**2 + dy**2 + dz**2) ** 0.5 < tol:
                break

            # После достижения обходной точки снова целимся в первоначальную цель.
            if active_target != goal:
                detour_dx = self.pose.pose.position.x - active_target[0]  # Ошибка до обходной точки по X.
                detour_dy = self.pose.pose.position.y - active_target[1]  # Ошибка до обходной точки по Y.
                detour_dz = self.pose.pose.position.z - active_target[2]  # Ошибка до обходной точки по Z.
                if (detour_dx**2 + detour_dy**2 + detour_dz**2) ** 0.5 < tol:
                    active_target = goal

        else:
            self.set_setpoint_target(
                self.pose.pose.position.x,
                self.pose.pose.position.y,
                self.pose.pose.position.z,
                yaw,
            )
            self.get_logger().error("Время ожидания достижения точки истекло.")
            if slowed_down:
                self.set_navigation_speed(NAV_SPEED_M_S)
            return False

        if slowed_down:
            self.set_navigation_speed(NAV_SPEED_M_S)

        if hold_sec > 0:
            self.get_logger().info(f"Holding position for {hold_sec}s...")
            self.spin_for(hold_sec)
        return True

    def land(self, timeout=150.0):
        # Останавливает setpoint-ы и отдаёт FCU команду посадки.
        self.get_logger().info("Landing...")
        self._sp_target = None
        self._vel_target = None
        while not self.land_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for land service...")
        req = CommandTOL.Request()  # Запрос FCU на выполнение посадки.
        future = self.land_cli.call_async(req)  # Асинхронный ответ сервиса посадки.
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None or not future.result().success:
            self.get_logger().warn("LAND service call failed, falling back to set_mode(LAND)")
            self.set_mode("LAND")