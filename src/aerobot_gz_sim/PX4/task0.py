#!/usr/bin/env python3
"""
Миссия для PX4 + ROS2 Humble + MAVROS (namespace /uav1) в Gazebo.

Сценарий:
  1. Проверка, что local_position/pose живая и адекватная.
  2. Offboard + arm, взлёт на 2 м (плавная рампа, контроль "ухода вверх").
  3. Полёт прямо по начальному курсу ~0.8 м/с; лидар /uav1/scan тормозит и
     отталкивает от стен. Нижняя камера ищет красную точку.
  4. Красная точка -> центрируемся -> висим 10 с -> домой -> посадка.
  5. Нет точки, но впереди стена -> домой -> посадка.

Защиты:
  * старт только если pose свежая, без NaN и стабильная;
  * в полёте: pose устарела / z выше потолка / z уходит выше уставки -> AUTO.LAND;
  * лидар пропал -> зависание, надолго -> посадка;
  * высота держится P-регулятором по z из local_position/pose.

Запуск (после source ~/Firmware2/install/setup.bash, симулятор с mavros запущен):
    python3 red_dot_mission_safe.py
"""
import collections
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import CommandBool, CommandLong, SetMode

# ---------------------------- ПАРАМЕТРЫ ----------------------------
NS = '/uav1'
TAKEOFF_ALT = 2.0
CEILING_ABOVE_START = 3.5
RUNAWAY_MARGIN = 1.0
CRUISE_SPEED = 0.8
MAX_SPEED = 1.0
TAKEOFF_RATE = 0.4
KZ = 1.0
MAX_VZ = 0.5

WALL_DIST = 5.5
STOP_DIST = 1.2
SLOW_DIST = 3.0
SIDE_DIST = 1.0
REPULSE_GAIN = 1.0
SECTOR_HALF = math.radians(30)
MIN_VALID_RANGE = 0.25

MAX_FORWARD_DIST = 80.0
RED_MIN_PIXELS = 25
RED_CENTER_TOL = 0.15
RED_CENTER_HOLD = 1.0
RED_LOST_TIMEOUT = 1.5
CENTER_KP = 0.8
CENTER_MAX_V = 0.5
HOVER_TIME = 10.0
HOME_TOL = 0.3

POSE_TIMEOUT = 2.0
SCAN_TIMEOUT_HOVER = 0.5
SCAN_TIMEOUT_LAND = 3.0

# Принудительное взведение (обходит preflight-проверки PX4). ТОЛЬКО для симулятора!
FORCE_ARM = True

# Нижняя камера: верх кадра = нос дрона, правая сторона кадра = правый борт.
# Если при центрировании дрон уходит не туда - поменяйте знак.
CAM_FWD_SIGN = -1.0
CAM_LEFT_SIGN = -1.0
DEFAULT_HFOV = 1.047


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class Mission(Node):
    def __init__(self):
        super().__init__('red_dot_mission')

        self.mav = None
        self.pose_ok = False
        self.pose_count = 0
        self.t_pose = 0.0
        self.x = self.y = self.z = self.yaw = 0.0
        self.scan = None
        self.t_scan = 0.0
        self.cam_k = None
        self.red = None
        self.t_red = 0.0
        self._warned_enc = False
        self.zhist = collections.deque()

        qos = qos_profile_sensor_data
        self.create_subscription(State, f'{NS}/mavros/state', self.cb_state, qos)
        self.create_subscription(StatusText, f'{NS}/mavros/statustext/recv', self.cb_status, qos)
        self.create_subscription(PoseStamped, f'{NS}/mavros/local_position/pose', self.cb_pose, qos)
        self.create_subscription(LaserScan, f'{NS}/scan', self.cb_scan, qos)
        self.create_subscription(Image, f'{NS}/camera_down', self.cb_img, qos)
        self.create_subscription(CameraInfo, f'{NS}/camera_down_info', self.cb_info, qos)

        self.pub_pos = self.create_publisher(PoseStamped, f'{NS}/mavros/setpoint_position/local', 10)
        self.pub_vel = self.create_publisher(TwistStamped, f'{NS}/mavros/setpoint_velocity/cmd_vel', 10)
        self.cli_arm = self.create_client(CommandBool, f'{NS}/mavros/cmd/arming')
        self.cli_mode = self.create_client(SetMode, f'{NS}/mavros/set_mode')
        self.cli_cmd = self.create_client(CommandLong, f'{NS}/mavros/cmd/command')

        self.state = 'INIT'
        self.t_state = time.monotonic()
        self.home = None
        self.z0 = 0.0
        self.yaw0 = 0.0
        self.z_ref = 0.0
        self.z_sp = 0.0
        self.t_last_req = 0.0
        self.center_since = None
        self.blocked_since = None
        self.prestream_n = 0
        self.last_log = 0.0

        self.create_timer(0.05, self.tick)  # 20 Гц

    # ------------------------- колбэки -------------------------
    def cb_state(self, msg):
        self.mav = msg

    def cb_status(self, msg):
        # сообщения PX4 (в т.ч. причины отказа arm: "Preflight Fail: ...")
        self.get_logger().warn(f'PX4[{msg.severity}]: {msg.text}')

    def cb_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        vals = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if any(not math.isfinite(v) for v in vals):
            self.pose_ok = False
            return
        self.x, self.y, self.z = p.x, p.y, p.z
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.t_pose = time.monotonic()
        self.pose_ok = True
        self.pose_count += 1
        self.zhist.append((self.t_pose, self.z))
        while self.zhist and self.t_pose - self.zhist[0][0] > 3.0:
            self.zhist.popleft()

    def pose_stable(self):
        """z не менялась больше 0.15 м за последние ~3 с (EKF "устоялся")."""
        if len(self.zhist) < 10 or self.zhist[-1][0] - self.zhist[0][0] < 2.5:
            return False
        zs = [z for _, z in self.zhist]
        return (max(zs) - min(zs)) < 0.15

    def cb_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float32)
        ang = msg.angle_min + np.arange(r.size, dtype=np.float32) * msg.angle_increment
        lo = max(msg.range_min, MIN_VALID_RANGE)
        valid = np.isfinite(r) & (r >= lo) & (r <= msg.range_max)
        r = np.where(valid, r, np.inf)
        self.scan = (ang, r)
        self.t_scan = time.monotonic()

    def cb_info(self, msg):
        if msg.k[0] > 0:
            self.cam_k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def cb_img(self, msg):
        if self.state not in ('FORWARD', 'CENTER', 'HOVER'):
            return
        enc = msg.encoding.lower()
        ch = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(enc, 0)
        if ch == 0:
            if not self._warned_enc:
                self.get_logger().warn(f'Неподдерживаемый формат изображения: {enc}')
                self._warned_enc = True
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        arr = arr[:, :msg.width * ch].reshape(msg.height, msg.width, ch)[::2, ::2]
        if enc.startswith('bgr'):
            b, g, r = arr[..., 0], arr[..., 1], arr[..., 2]
        else:
            r, g, b = arr[..., 0], arr[..., 1], arr[..., 2]
        mask = (r > 150) & (g < 90) & (b < 90)
        if int(mask.sum()) < RED_MIN_PIXELS:
            return
        ys, xs = np.nonzero(mask)
        u, v = float(xs.mean()) * 2, float(ys.mean()) * 2
        if self.cam_k:
            fx, fy, cx, cy = self.cam_k
        else:
            fx = fy = msg.width / (2 * math.tan(DEFAULT_HFOV / 2))
            cx, cy = msg.width / 2, msg.height / 2
        h = max(self.z - self.z0, 0.3)
        self.red = (CAM_FWD_SIGN * (v - cy) / fy * h, CAM_LEFT_SIGN * (u - cx) / fx * h)
        self.t_red = time.monotonic()

    # ------------------------- утилиты -------------------------
    def log(self, text, period=1.0):
        now = time.monotonic()
        if now - self.last_log > period:
            self.get_logger().info(text)
            self.last_log = now

    def goto(self, state):
        self.get_logger().info(f'Состояние: {self.state} -> {state}')
        self.state = state
        self.t_state = time.monotonic()
        self.center_since = None
        self.blocked_since = None

    def request_mode(self, mode):
        if self.cli_mode.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = mode
            fut = self.cli_mode.call_async(req)
            fut.add_done_callback(
                lambda f, m=mode: self.get_logger().info(
                    f'set_mode({m}): mode_sent={f.result().mode_sent}' if f.result()
                    else f'set_mode({m}): нет ответа'))
        else:
            self.get_logger().warn('Сервис set_mode недоступен')

    def request_arm(self):
        if FORCE_ARM and self.cli_cmd.service_is_ready():
            req = CommandLong.Request()
            req.command = 400        # MAV_CMD_COMPONENT_ARM_DISARM
            req.param1 = 1.0         # arm
            req.param2 = 21196.0     # force (магическое число MAVLink)
            fut = self.cli_cmd.call_async(req)
            fut.add_done_callback(
                lambda f: self.get_logger().info(
                    f'force arm: success={f.result().success} result={f.result().result}'
                    if f.result() else 'force arm: нет ответа'))
            return
        if self.cli_arm.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            fut = self.cli_arm.call_async(req)
            fut.add_done_callback(
                lambda f: self.get_logger().info(
                    f'arming: success={f.result().success} result={f.result().result}' if f.result()
                    else 'arming: нет ответа'))
        else:
            self.get_logger().warn('Сервис arming недоступен')

    def pose_fresh(self):
        return self.pose_ok and (time.monotonic() - self.t_pose) < POSE_TIMEOUT

    def pub_position(self, x, y, z):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        m.pose.orientation.z = math.sin(self.yaw0 / 2)
        m.pose.orientation.w = math.cos(self.yaw0 / 2)
        self.pub_pos.publish(m)

    def pub_velocity(self, vx, vy, vz=None):
        if vz is None:
            vz = clamp(KZ * (self.z_ref - self.z), -MAX_VZ, MAX_VZ)
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = float(vx), float(vy), float(vz)
        self.pub_vel.publish(m)

    # ------------------------- лидар -------------------------
    def clearance(self, world_angle):
        if self.scan is None:
            return 0.0
        ang, r = self.scan
        d = np.abs((ang - (world_angle - self.yaw) + np.pi) % (2 * np.pi) - np.pi)
        sel = r[d < SECTOR_HALF]
        return float(sel.min()) if sel.size else float('inf')

    def repulsion(self):
        if self.scan is None:
            return 0.0, 0.0
        ang, r = self.scan
        close = np.isfinite(r) & (r < SIDE_DIST)
        if not close.any():
            return 0.0, 0.0
        w = (SIDE_DIST - r[close]) / SIDE_DIST
        bx = -float(np.mean(w * np.cos(ang[close]))) * REPULSE_GAIN
        by = -float(np.mean(w * np.sin(ang[close]))) * REPULSE_GAIN
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return bx * c - by * s, bx * s + by * c

    def safe_velocity(self, dx, dy, speed):
        clear = self.clearance(math.atan2(dy, dx))
        k = clamp((clear - STOP_DIST) / (SLOW_DIST - STOP_DIST), 0.0, 1.0)
        vx, vy = dx * speed * k, dy * speed * k
        rx, ry = self.repulsion()
        vx += rx
        vy += ry
        n = math.hypot(vx, vy)
        if n > MAX_SPEED:
            vx, vy = vx / n * MAX_SPEED, vy / n * MAX_SPEED
        return vx, vy, clear

    # ------------------------- безопасность -------------------------
    def emergency(self, reason):
        self.get_logger().error(f'АВАРИЙНАЯ ПОСАДКА: {reason}')
        self.goto('EMERGENCY')
        self.request_mode('AUTO.LAND')

    def safety_check(self):
        if not self.pose_fresh():
            return 'local_position/pose устарела или содержит NaN'
        if self.z > self.z0 + CEILING_ABOVE_START:
            return f'превышен потолок: z={self.z:.2f}'
        if self.state == 'TAKEOFF' and self.z > self.z_sp + RUNAWAY_MARGIN:
            return f'дрон уходит вверх быстрее уставки: z={self.z:.2f}, уставка={self.z_sp:.2f}'
        if self.state in ('FORWARD', 'CENTER', 'HOVER', 'RETURN') and \
                self.z > self.z_ref + RUNAWAY_MARGIN:
            return f'уход вверх от заданной высоты: z={self.z:.2f}'
        if self.mav is None or not self.mav.connected:
            return 'потеряна связь с FCU'
        if self.scan is None or (time.monotonic() - self.t_scan) > SCAN_TIMEOUT_LAND:
            return 'нет данных лидара'
        return None

    # ------------------------- главный цикл -------------------------
    def tick(self):
        now = time.monotonic()
        S = self.state

        if S == 'INIT':
            ready = (self.mav is not None and self.mav.connected and self.pose_fresh()
                     and self.pose_count >= 40 and self.scan is not None
                     and self.cli_arm.service_is_ready() and self.cli_mode.service_is_ready())
            self.log('Жду MAVROS, local_position, лидар и сервисы...')
            if not ready:
                return
            if not self.pose_stable():
                self.log('Позиция ещё "плывёт" (EKF не устоялся), жду...')
                return
            if True:
                self.home = (self.x, self.y)
                self.z0 = self.z
                self.yaw0 = self.yaw
                self.get_logger().info(
                    f'Старт: x={self.x:.2f} y={self.y:.2f} z={self.z:.2f} yaw={math.degrees(self.yaw):.0f}°')
                # Абсолютный z не важен: начало координат EKF по высоте произвольно.
                # Вся миссия считает высоту относительно z0 (стартовая точка на земле).
                self.get_logger().info(f'Нулевая высота (z0) = {self.z0:.2f} м, потолок = {self.z0 + CEILING_ABOVE_START:.2f} м')
                self.z_sp = self.z0
                self.z_ref = self.z0 + TAKEOFF_ALT
                self.goto('PRESTREAM')
            return

        if S == 'PRESTREAM':
            self.pub_position(self.home[0], self.home[1], self.z0)
            self.prestream_n += 1
            if self.prestream_n > 40:
                self.goto('ARMING')
            return

        if S in ('ARMING', 'TAKEOFF', 'FORWARD', 'CENTER', 'HOVER', 'RETURN'):
            if S != 'ARMING':
                reason = self.safety_check()
                if reason:
                    self.emergency(reason)
                    return
                if self.mav.mode != 'OFFBOARD':
                    self.get_logger().warn(f'Режим сменён на {self.mav.mode}, миссия остановлена.')
                    self.goto('DONE')
                    return
            elif not self.pose_fresh():
                self.log('ARMING: pose устарела, жду')
                return

        if S == 'ARMING':
            self.pub_position(self.home[0], self.home[1], self.z0)
            if self.mav.mode == 'OFFBOARD' and self.mav.armed:
                self.goto('TAKEOFF')
                return
            self.log(f'ARMING: mode={self.mav.mode} armed={self.mav.armed} '
                     f'connected={self.mav.connected}')
            if now - self.t_last_req > 2.0:
                self.t_last_req = now
                if self.mav.mode != 'OFFBOARD':
                    self.request_mode('OFFBOARD')
                if not self.mav.armed:
                    self.request_arm()
            if now - self.t_state > 60.0:
                self.get_logger().error('Не удалось войти в OFFBOARD/arm за 60 с - отмена.')
                self.goto('DONE')
            return

        if S == 'TAKEOFF':
            self.z_sp = min(self.z_sp + TAKEOFF_RATE * 0.05, self.z_ref)
            self.pub_position(self.home[0], self.home[1], self.z_sp)
            self.log(f'Взлёт: z={self.z - self.z0:.2f} м, уставка={self.z_sp - self.z0:.2f} м')
            if abs(self.z - self.z_ref) < 0.15 and self.z_sp >= self.z_ref:
                if self.center_since is None:
                    self.center_since = now
                elif now - self.center_since > 1.0:
                    self.goto('FORWARD')
            else:
                self.center_since = None
            if now - self.t_state > 120.0:
                self.emergency('взлёт не завершён за 120 с')
            return

        if S in ('FORWARD', 'CENTER', 'HOVER', 'RETURN') and \
                (now - self.t_scan) > SCAN_TIMEOUT_HOVER:
            self.pub_velocity(0.0, 0.0)
            self.log('Нет свежих данных лидара - зависаю')
            return

        red_fresh = self.red is not None and (now - self.t_red) < 0.5

        if S == 'FORWARD':
            fdx, fdy = math.cos(self.yaw0), math.sin(self.yaw0)
            if red_fresh:
                self.goto('CENTER')
                return
            vx, vy, clear = self.safe_velocity(fdx, fdy, CRUISE_SPEED)
            self.pub_velocity(vx, vy)
            dist = math.hypot(self.x - self.home[0], self.y - self.home[1])
            self.log(f'Вперёд: впереди {clear:.1f} м, пройдено {dist:.1f} м')
            if clear < WALL_DIST:
                self.get_logger().info('Стена впереди, красной точки нет -> домой')
                self.goto('RETURN')
            elif dist > MAX_FORWARD_DIST:
                self.get_logger().info('Предельная дистанция -> домой')
                self.goto('RETURN')
            return

        if S in ('CENTER', 'HOVER'):
            rx, ry = self.repulsion()
            if red_fresh:
                f, l = self.red
                vf = clamp(CENTER_KP * f, -CENTER_MAX_V, CENTER_MAX_V)
                vl = clamp(CENTER_KP * l, -CENTER_MAX_V, CENTER_MAX_V)
                c, s = math.cos(self.yaw), math.sin(self.yaw)
                self.pub_velocity(vf * c - vl * s + rx, vf * s + vl * c + ry)
                centered = math.hypot(f, l) < RED_CENTER_TOL
            else:
                self.pub_velocity(rx, ry)
                centered = False
                if S == 'CENTER' and (now - self.t_red) > RED_LOST_TIMEOUT:
                    self.get_logger().info('Красная точка потеряна -> продолжаю вперёд')
                    self.goto('FORWARD')
                    return
            if S == 'CENTER':
                self.log(f'Центрируюсь: смещение {self.red}')
                if centered:
                    self.center_since = self.center_since or now
                    if now - self.center_since > RED_CENTER_HOLD:
                        self.get_logger().info('Над красной точкой, жду 10 с')
                        self.goto('HOVER')
                else:
                    self.center_since = None
            else:
                left = HOVER_TIME - (now - self.t_state)
                self.log(f'Висим над точкой, осталось {left:.0f} с')
                if left <= 0:
                    self.goto('RETURN')
            return

        if S == 'RETURN':
            ex, ey = self.home[0] - self.x, self.home[1] - self.y
            dist = math.hypot(ex, ey)
            if dist < HOME_TOL:
                self.goto('LANDING')
                self.request_mode('AUTO.LAND')
                return
            speed = min(CRUISE_SPEED, 0.6 * dist + 0.15)
            vx, vy, clear = self.safe_velocity(ex / dist, ey / dist, speed)
            self.pub_velocity(vx, vy)
            self.log(f'Домой: {dist:.1f} м, впереди {clear:.1f} м')
            if clear < STOP_DIST + 0.2:
                self.blocked_since = self.blocked_since or now
                if now - self.blocked_since > 15.0:
                    self.get_logger().warn('Путь домой заблокирован, сажусь на месте')
                    self.goto('LANDING')
                    self.request_mode('AUTO.LAND')
            else:
                self.blocked_since = None
            return

        if S in ('LANDING', 'EMERGENCY'):
            if self.mav is not None and self.mav.mode != 'AUTO.LAND' and now - self.t_last_req > 1.0:
                self.t_last_req = now
                self.request_mode('AUTO.LAND')
            self.log(f'Посадка, высота {self.z - self.z0:.2f} м')
            if self.pose_fresh() and (self.z - self.z0) < 0.15 and now - self.t_state > 3.0:
                self.get_logger().info('Приземлились. Миссия завершена.')
                self.goto('DONE')
            return

        if S == 'DONE':
            self.log('Готово. Ctrl+C для выхода.', period=5.0)


def main():
    rclpy.init()
    node = Mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.state not in ('INIT', 'PRESTREAM', 'DONE'):
            node.request_mode('AUTO.LAND')
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
