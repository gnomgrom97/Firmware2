"""Проход по комнатам по QR-кодам и посадка на платформу с кодом маршрута.

Управление полётом выполняет UavControllerArduPilot. Этот файл отвечает за
получение изображений с камер, распознавание кодов, оценку направления дверей,
сбор маршрута и передачу контроллеру очередных координатных целей.
"""

import math
import time

import rclpy
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
from sensor_msgs.msg import CameraInfo, Image

from uav_controller_ardupilot import UavControllerArduPilot

# Постоянная высота полёта и параметры осмотра комнаты.
FLIGHT_ALTITUDE = 2.0  # Рабочая высота и высота координат целей, метры.
SURVEY_YAW_RATE = 0.25  # Угловая скорость поворота при круговом осмотре, рад/с.
ROOM_SCAN_SECONDS = 27.0  # Время обзора комнаты; при заданной скорости это чуть больше одного оборота.
DOOR_CROSS_DISTANCE_M = 4.0  # Длина шага через дверь в направлении её QR-кода, метры.
MAX_ROOM_TRANSITIONS = 10  # Защита от бесконечного обхода при ошибочных или повторяющихся кодах.
# Смещения для поиска платформы вокруг места, где она была замечена в последний раз.
# Первая точка совпадает с исходной позицией, остальные образуют сетку вокруг неё.
PLATFORM_SCAN_OFFSETS_M = (
    (0.0, 0.0), (1.5, 0.0), (-1.5, 0.0), (0.0, 1.5), (0.0, -1.5),
    (1.5, 1.5), (1.5, -1.5), (-1.5, 1.5), (-1.5, -1.5),
)

# Резервная модель камеры нужна до получения сообщения CameraInfo или если оно
# не содержит корректных фокусных расстояний. Здесь предполагается камера
# 1280x720 пикселей с горизонтальным полем зрения 1.74 радиана.
CAM_FX_FALLBACK = 1280.0 / (2.0 * math.tan(1.74 / 2.0))
CAM_FY_FALLBACK = CAM_FX_FALLBACK
# Главная точка изображения: центр кадра в пикселях (cx, cy).
CAM_CX_FALLBACK, CAM_CY_FALLBACK = 640.0, 360.0


class QrDetector:
    """Распознаёт QR-коды и хранит их центры в кадре и параметры камеры.

    Для каждого нового изображения список codes заменяется результатами
    именно этого кадра. Элементы списка имеют вид (текст кода, x, y), где x и y
    — координаты центра QR в пикселях. Из-за такого хранения потребитель всегда
    видит последний обработанный кадр, а не историю всех обнаружений.
    """

    def __init__(self, node, topic):
        self.bridge = CvBridge()  # Конвертер ROS Image в массив изображения OpenCV.
        self.codes = []  # QR-коды, найденные в последнем обработанном кадре.
        self.frame_number = 0  # Число обработанных кадров; удобно для диагностики.
        # Пока не получена CameraInfo, используем расчётные параметры симуляционной камеры.
        self.fx = CAM_FX_FALLBACK
        self.fy = CAM_FY_FALLBACK
        self.cx = CAM_CX_FALLBACK
        self.cy = CAM_CY_FALLBACK
        # Изображение несёт пиксели, CameraInfo — внутреннюю матрицу калибровки.
        # Суффикс _info строится из имени соответствующего топика изображения.
        node.create_subscription(Image, topic, self._cb, 10)
        node.create_subscription(CameraInfo, f"{topic}_info", self._info_cb, 10)

    def _info_cb(self, msg):
        # В матрице K фокусные расстояния находятся на главной диагонали,
        # а координаты главной точки — в элементах K[2] и K[5]. Нулевые или
        # отрицательные фокусные расстояния означают, что калибровка непригодна.
        if msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]

    def _cb(self, msg):
        # pyzbar ожидает обычное изображение; приводим вход к трёхканальному BGR.
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # Старые обнаружения намеренно удаляются, чтобы не использовать QR,
        # который исчез из поля зрения в новом кадре.
        self.codes = []
        for result in decode(frame):
            # Обычно pyzbar возвращает углы контура. Для результата без контура
            # вычисляем тот же центр по ограничивающему прямоугольнику.
            if result.polygon:
                center_x = sum(point.x for point in result.polygon) / len(result.polygon)
                center_y = sum(point.y for point in result.polygon) / len(result.polygon)
            else:
                rect = result.rect
                center_x = rect.left + rect.width / 2.0
                center_y = rect.top + rect.height / 2.0
            # Декодированные байты переводятся в текст; strip убирает пробелы
            # и переносы по краям, чтобы сравнение кодов было стабильным.
            self.codes.append((result.data.decode("utf-8").strip(), center_x, center_y))
        # Счётчик увеличивается после обработки кадра, в том числе если QR не найден.
        self.frame_number += 1


def scan_room(uav, qr_down, qr_front, duration=ROOM_SCAN_SECONDS):
    """Осматривает комнату и возвращает напольные QR и направления QR дверей.

    QR нижней камеры сохраняется вместе с позой и курсом дрона в момент
    обнаружения: эти данные позже нужны для оценки координат кода на полу.
    Для передней камеры рассчитывается мировой азимут на каждый QR двери.
    """
    floor_codes = {}  # код -> пиксельный центр, позиция дрона и его курс при наблюдении.
    door_bearings = {}  # код двери -> оценённый мировой угол направления на неё, радианы.
    # monotonic не зависит от коррекции системных часов и надёжен для таймаута.
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        # Дрон вращается на месте: линейные скорости равны нулю, задаётся только yaw_rate.
        uav.set_velocity_target(0.0, 0.0, 0.0, SURVEY_YAW_RATE)
        # Обрабатываем ROS-сообщения, чтобы обновились поза, камеры и состояние контроллера.
        rclpy.spin_once(uav, timeout_sec=0.05)

        # Курс переводит направление из системы камеры в мировое направление.
        yaw = uav._yaw_from_pose()
        position = uav.pose.pose.position
        for code, px, py in qr_down.codes:
            # Повторное обнаружение того же кода обновляет запись; в итоге остаётся
            # последнее наблюдение за время сканирования.
            floor_codes[code] = (px, py, position.x, position.y, position.z, yaw)
        for code, px, _ in qr_front.codes:
            # Смещение по горизонтали относительно центра кадра задаёт угол луча
            # относительно оптической оси: atan2(смещение пикселей, фокусное расстояние).
            bearing_offset = math.atan2(px - qr_front.cx, qr_front.fx)
            # Добавляем поворот дрона, получая азимут в мировой системе координат.
            door_bearings[code] = yaw + bearing_offset

    # После осмотра прекращаем вращение и даём контроллеру короткое время
    # обработать команду остановки до возврата результатов вызывающему коду.
    uav.stop()
    uav.spin_for(0.3)
    # Выводим сами коды в журнал, чтобы было видно, что распознала каждая камера.
    for code in floor_codes:
        uav.get_logger().info(f"[QR floor] {code}")
    for code in door_bearings:
        uav.get_logger().info(f"[QR door] {code}")
    return floor_codes, door_bearings


def floor_qr_world_position(observation, qr_down):
    """Оценивает мировые координаты QR-кода на полу по нижней камере.

    observation содержит пиксельный центр QR и позу дрона в момент кадра.
    Используется приближение камеры, направленной вертикально вниз: луч от
    центра изображения к точке QR пересекает плоскость пола на расстоянии,
    пропорциональном высоте и смещению пикселя от главной точки.
    """
    px, py, drone_x, drone_y, drone_z, yaw = observation
    # Отрицательная высота физически невозможна для этой оценки; на земле
    # расстояние от камеры до пола принимается равным нулю.
    height = max(0.0, drone_z)
    # При вертикальной камере масштаб на плоскости равен высота / фокусное
    # расстояние. Знаки выбраны для осей изображения (x вправо, y вниз) и
    # соглашения контроллера: forward — вперёд, left — влево.
    forward = -(py - qr_down.cy) * height / qr_down.fy
    left = -(px - qr_down.cx) * height / qr_down.fx
    # Поворот локальных смещений на yaw и перенос в мировую систему координат.
    world_x = drone_x + forward * math.cos(yaw) - left * math.sin(yaw)
    world_y = drone_y + forward * math.sin(yaw) + left * math.cos(yaw)
    # Цель задаётся на рабочей высоте: поиск точки на полу не означает посадку.
    return world_x, world_y, FLIGHT_ALTITUDE


def find_route_platform(uav, qr_down, qr_front, route_code):
    """Ищет платформу с собранным кодом маршрута по соседним позициям.

    Дрон облетает заданные смещения вокруг текущей точки, на каждой точке
    выполняет короткий осмотр нижней камерой и сравнивает найденные коды с
    route_code. При совпадении пересчитывает координаты платформы и летит к ней.
    """
    origin = uav.pose.pose.position
    origin_x, origin_y = origin.x, origin.y
    for offset_x, offset_y in PLATFORM_SCAN_OFFSETS_M:
        # Каждое смещение отсчитывается от одной и той же исходной позиции,
        # а не от предыдущей точки поиска.
        scan_x, scan_y = origin_x + offset_x, origin_y + offset_y
        # Если waypoint недостижим, нет смысла сканировать эту позицию.
        if not uav.goto(scan_x, scan_y, FLIGHT_ALTITUDE, hold_sec=0.2, tol=0.3):
            continue
        # Короткого вращения достаточно для проверки платформы рядом с точкой.
        floor_codes, _ = scan_room(uav, qr_down, qr_front, duration=1.0)
        observation = floor_codes.get(route_code)
        if observation is None:
            continue

        # QR найден: переводим его пиксельный центр в мировую координату пола.
        target = floor_qr_world_position(observation, qr_down)
        uav.get_logger().info(f"Совпала платформа с кодом {route_code}: {target}")
        # Успешное достижение цели позволяет посадиться и завершить поиск.
        if uav.goto(*target, tol=0.3):
            uav.land()
            return True
    # Все точки сетки проверены, но подходящая и достижимая цель не найдена.
    return False


def main():
    # ROS 2 должен быть инициализирован до создания узла и подписок.
    rclpy.init()
    uav = UavControllerArduPilot("uav1")

    # Камеры направлены вниз (QR пола) и вперёд (QR двери); подписки принадлежат узлу.
    qr_down = QrDetector(uav, "/uav1/camera_down")
    qr_front = QrDetector(uav, "/uav1/camera")
    route = []  # Последовательность номеров комнат; её конкатенация — код платформы.
    visited_codes = set()  # Не даёт повторно выбрать уже обработанный номер комнаты.

    try:
        # До навигации контроллер переводит аппарат в GUIDED, армит и выполняет взлёт.
        if not uav.run_mission(altitude=FLIGHT_ALTITUDE):
            uav.get_logger().error("Взлёт не удался, завершаю миссию.")
            return

        # Каждая итерация находит следующий номер комнаты и дверь с тем же QR-кодом.
        for _ in range(MAX_ROOM_TRANSITIONS):
            floor_codes, door_bearings = scan_room(uav, qr_down, qr_front)
            # Код платформы — маршрут, записанный подряд, например ["2", "4"] -> "24".
            route_code = "".join(route)

            # Проверяем целевую платформу до поиска следующей комнаты: она может
            # быть уже видна в текущем помещении.
            if route_code and route_code in floor_codes:
                target = floor_qr_world_position(floor_codes[route_code], qr_down)
                uav.get_logger().info(f"Найдена платформа {route_code}: {target}")
                if uav.goto(*target, tol=0.3):
                    uav.land()
                    return
                uav.get_logger().error("Не удалось подлететь к совпавшей платформе.")

            # Из кодов пола выбираем первую ещё не посещённую цифровую метку.
            # Номера, уже добавленные в маршрут, не должны создавать цикл.
            next_code = next(
                (code for code in floor_codes if code.isdigit() and code not in visited_codes),
                None,
            )
            if next_code is None:
                # Нового номера нет: вероятно, маршрут достиг конца или текущий
                # кадр не содержит меток. Пробуем локальный поиск платформы.
                uav.get_logger().warn(
                    "Новый QR комнаты не найден; ищу платформу по собранному маршруту."
                )
                if route_code and find_route_platform(uav, qr_down, qr_front, route_code):
                    return
                break

            # Сохраняем номер до перехода, чтобы при следующем сканировании не
            # принять ту же комнату за новую.
            visited_codes.add(next_code)
            route.append(next_code)
            uav.get_logger().info(f"Маршрут: {route} (код {''.join(route)})")

            # Направление двери берётся из передней камеры и привязано к тому же
            # номеру, что QR комнаты, найденный нижней камерой.
            door_bearing = door_bearings.get(next_code)
            if door_bearing is None:
                # Код комнаты мог быть виден раньше, чем камера успела увидеть дверь.
                # Повторный осмотр даёт шанс получить направление для этого номера.
                uav.get_logger().warn(
                    f"QR двери {next_code} не виден; повторно сканирую комнату."
                )
                floor_codes, door_bearings = scan_room(uav, qr_down, qr_front)
                door_bearing = door_bearings.get(next_code)
            if door_bearing is None:
                uav.get_logger().error(f"Не найден проём с QR {next_code}.")
                break

            # Строим waypoint на фиксированном расстоянии по азимуту двери.
            # Угол используется и для направления движения, и для ориентации дрона.
            position = uav.pose.pose.position
            target = (
                position.x + DOOR_CROSS_DISTANCE_M * math.cos(door_bearing),
                position.y + DOOR_CROSS_DISTANCE_M * math.sin(door_bearing),
                FLIGHT_ALTITUDE,
            )
            uav.get_logger().info(f"Перехожу в проём {next_code}: {target}")
            if not uav.goto(*target, yaw=door_bearing, tol=0.4):
                uav.get_logger().error(f"Не удалось пройти проём {next_code}.")
                break

        # Цикл закончился без посадки на платформу либо был прерван из-за
        # отсутствующих QR/недостижимой двери; безопасно завершаем миссию посадкой.
        uav.get_logger().error(
            f"Миссия не завершена; собранный маршрут: {route}. "
            "Сажусь на текущей позиции."
        )
        uav.land()
    finally:
        # Очистка выполняется и при раннем return, и при исключении: освобождаем
        # ROS-узел и завершаем клиентскую часть ROS 2.
        uav.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    # Не запускаем миссию при импорте этого файла как модуля.
    main()