import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from uav_controller_ardupilot import UavControllerArduPilot

FLIGHT_ALTITUDE = 2.0
SEARCH_SPEED_M_S = 1.5
HOLD_SECONDS = 10.0
POSITION_TOLERANCE_M = 0.2
RED_CIRCLE_TOLERANCE_M = 0.05
SEARCH_TIMEOUT_S = 60.0
RETURN_STABLE_SAMPLES = 10
PRECISION_SPEED_M_S = 0.5

# Fallback calibration for the 1280x720 camera model.
CAM_FX = 1280.0 / (2.0 * math.tan(1.74 / 2.0))
CAM_FY = CAM_FX
CAM_CX, CAM_CY = 640.0, 360.0
MIN_CIRCLE_AREA = 300
MIN_CIRCULARITY = 0.65


class DownwardCamera:
    def __init__(self, node):
        self.bridge = CvBridge()
        self.frame = None
        self.last_frame_time = 0.0
        self.frame_number = 0
        self.fx = CAM_FX
        self.fy = CAM_FY
        self.cx = CAM_CX
        self.cy = CAM_CY
        node.create_subscription(Image, "/uav1/camera_down", self._callback, 10)
        node.create_subscription(
            CameraInfo, "/uav1/camera_down_info", self._info_callback, 10
        )

    def _info_callback(self, msg):
        if msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]

    def _callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.last_frame_time = time.monotonic()
        self.frame_number += 1


def find_red_circle(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = cv2.inRange(hsv, np.array((0, 100, 100)), np.array((10, 255, 255)))
    upper_red = cv2.inRange(hsv, np.array((170, 100, 100)), np.array((180, 255, 255)))
    mask = cv2.bitwise_or(lower_red, upper_red)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    circles = []
    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if area < MIN_CIRCLE_AREA or perimeter == 0:
            continue
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        if circularity < MIN_CIRCULARITY:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        center = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
        circles.append((area, center))

    return max(circles, default=(0, None), key=lambda item: item[0])[1]


def estimate_circle_position(pixel, camera, uav, yaw):
    px, py = pixel
    position = uav.pose.pose.position
    height = max(0.0, position.z)
    # The simulated camera is pitched down by 90 degrees: image-down points
    # backward and image-right points to the drone's right side.
    forward = -(py - camera.cy) * height / camera.fy
    left = -(px - camera.cx) * height / camera.fx
    offset_x = forward * math.cos(yaw) - left * math.sin(yaw)
    offset_y = forward * math.sin(yaw) + left * math.cos(yaw)
    return position.x + offset_x, position.y + offset_y, position.z


def wait_for_camera_frame(uav, camera, timeout=5.0):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and camera.frame is None and time.monotonic() < deadline:
        rclpy.spin_once(uav, timeout_sec=0.1)
    return camera.frame is not None


def fly_to_position(uav, target, yaw, timeout=120.0, tolerance=POSITION_TOLERANCE_M):
    uav.set_setpoint_target(*target, yaw)
    deadline = time.monotonic() + timeout
    stable_samples = 0
    while rclpy.ok() and time.monotonic() < deadline:
        previous_pose = uav.pose
        rclpy.spin_once(uav, timeout_sec=0.05)
        if uav.pose is previous_pose:
            continue
        position = uav.pose.pose.position
        distance = math.dist((position.x, position.y, position.z), target)
        stable_samples = stable_samples + 1 if distance <= tolerance else 0
        if stable_samples >= RETURN_STABLE_SAMPLES:
            return True
    return False


def target_from_circle(uav, camera, pixel, yaw):
    target = estimate_circle_position(pixel, camera, uav, yaw)
    return target[0], target[1], FLIGHT_ALTITUDE


def search_straight(uav, camera, yaw):
    uav.get_logger().info(
        f"Лечу прямо со скоростью {SEARCH_SPEED_M_S:.1f} м/с "
        f"и ищу красный круг (таймаут {SEARCH_TIMEOUT_S:.0f} с)."
    )
    uav.set_velocity_target(
        SEARCH_SPEED_M_S * math.cos(yaw),
        SEARCH_SPEED_M_S * math.sin(yaw),
        0.0,
        0.0,
    )
    deadline = time.monotonic() + SEARCH_TIMEOUT_S

    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(uav, timeout_sec=0.05)
        if camera.frame is None or time.monotonic() - camera.last_frame_time > 1.0:
            continue
        pixel = find_red_circle(camera.frame)
        if pixel is None:
            continue

        position = uav.pose.pose.position
        uav.set_setpoint_target(position.x, position.y, position.z, yaw)
        if not uav.set_navigation_speed(PRECISION_SPEED_M_S):
            uav.get_logger().error("Не удалось снизить скорость для точного наведения.")
            return False
        target = target_from_circle(uav, camera, pixel, yaw)
        uav.get_logger().info(
            f"Найден красный круг; лечу над ним в точку "
            f"({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f})."
        )
        if not fly_to_position(
            uav, target, yaw, tolerance=RED_CIRCLE_TOLERANCE_M
        ):
            uav.get_logger().error("Не удалось долететь к предварительной точке над кругом.")
            return False

        uav.set_setpoint_target(*target, yaw)
        uav.get_logger().info("Дрон над красной точкой, начинаю отсчёт 10 секунд.")
        uav.spin_for(HOLD_SECONDS)
        return True

    uav.get_logger().warn("Красный круг не найден на прямом маршруте.")
    return False


def main():
    rclpy.init()
    uav = UavControllerArduPilot("uav1")
    camera = DownwardCamera(uav)

    try:
        if not uav.run_mission(altitude=FLIGHT_ALTITUDE):
            uav.get_logger().error("Взлёт не удался, завершаю миссию.")
            return

        if not uav.set_navigation_speed(SEARCH_SPEED_M_S):
            uav.get_logger().warn(
                "Не удалось установить скорость поиска; продолжаю с текущим ограничением."
            )

        position = uav.pose.pose.position
        start = (position.x, position.y, position.z)
        yaw = uav._yaw_from_pose()

        if not wait_for_camera_frame(uav, camera):
            uav.get_logger().error("Нет изображения с камеры, поиск отменён.")
        else:
            search_straight(uav, camera, yaw)

        if not uav.set_navigation_speed(SEARCH_SPEED_M_S):
            uav.get_logger().warn("Возвращаюсь с текущим ограничением скорости.")
        if not fly_to_position(uav, start, yaw, tolerance=0.1):
            uav.get_logger().error("Не удалось вернуться в стартовую точку.")
        uav.land()
    finally:
        uav.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
