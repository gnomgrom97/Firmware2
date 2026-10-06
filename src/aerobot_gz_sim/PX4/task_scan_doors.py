#!/usr/bin/env python3
"""
Миссия:
  1. Взлёт на 2.5 м.
  2. Полёт вперёд, поиск красного объекта нижней камерой.
  3. Медленный подлёт к цели.
  4. Стабилизация 1.5 сек.
  5. Зависание над точкой 10 сек.
  6. Медленный возврат на старт, мягкая посадка.
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, Image
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode

# ---------------------- ПАРАМЕТРЫ ----------------------
NS = '/uav1'
TAKEOFF_ALT = 2.5
TAKEOFF_RATE = 0.4
POSE_TIMEOUT = 2.0
FORCE_ARM = True
ALT_CEILING_MARGIN = 0.5

# Поиск
CRUISE_SPEED = 0.3
MAX_SEARCH_TIME = 60.0
WALL_STOP_DIST = 1.2

# Подлёт к цели
APPROACH_SPEED = 0.10
DRONE_POS_TOL = 0.12
APPROACH_TIMEOUT = 20.0

# Стабилизация и зависание
SETTLE_TIME = 1.5
HOVER_TIME = 10.0

# Возврат
HOME_TOL = 0.25
RETURN_SPEED = 0.15           # максимальная скорость горизонтального возврата
LAND_APPROACH_DIST = 1.0      # зона замедления у точки посадки

# Красный объект
RED_MIN_PIXELS = 40
RED_R_MIN = 150
RED_G_MAX = 100
RED_B_MAX = 100
CAM_HFOV = 1.755

# Буфер детекций
RED_BUFFER_SIZE = 5
RED_BUFFER_TIMEOUT = 2.0
RED_LOCK_DETECTIONS = 3
# -------------------------------------------------------


class SearchMission(Node):
    def __init__(self):
        super().__init__('search_mission')
        self.mav = None
        self.pose_ok = False
        self.pose_count = 0
        self.t_pose = 0.0
        self.x = self.y = self.z = self.yaw = 0.0
        self.scan = None
        self.t_scan = 0.0

        self.red_buffer = []
        self.red = None
        self.t_red = 0.0

        qos = qos_profile_sensor_data
        self.create_subscription(State, f'{NS}/mavros/state', self.cb_state, qos)
        self.create_subscription(PoseStamped, f'{NS}/mavros/local_position/pose', self.cb_pose, qos)
        self.create_subscription(LaserScan, f'{NS}/scan', self.cb_scan, qos)
        self.create_subscription(Image, f'{NS}/camera_down', self.cb_cam_down, qos)
        self.pub_pos = self.create_publisher(PoseStamped, f'{NS}/mavros/setpoint_position/local', 10)
        self.cli_arm = self.create_client(CommandBool, f'{NS}/mavros/cmd/arming')
        self.cli_mode = self.create_client(SetMode, f'{NS}/mavros/set_mode')
        self.cli_cmd = self.create_client(CommandLong, f'{NS}/mavros/cmd/command')

        self.state = 'INIT'
        self.t_state = time.monotonic()
        self.home = None
        self.z0 = 0.0
        self.yaw0 = 0.0
        self.z_sp = 0.0
        self.z_ref = 0.0
        self.tx = 0.0
        self.ty = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.t_last_req = 0.0
        self.prestream_n = 0
        self.last_log = 0.0
        self.create_timer(0.02, self.tick)

    # ---------------- КОЛБЭКИ ----------------
    def cb_state(self, msg):
        self.mav = msg

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

    def cb_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float32)
        ang = msg.angle_min + np.arange(r.size, dtype=np.float32) * msg.angle_increment
        lo = max(msg.range_min, 0.2)
        valid = np.isfinite(r) & (r >= lo)
        r = np.where(valid, r, np.inf)
        self.scan = (ang, r)
        self.t_scan = time.monotonic()

    def cb_cam_down(self, msg):
        if self.state != 'SEARCH':
            return
        enc = msg.encoding.lower()
        ch = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(enc, 0)
        if ch == 0:
            return
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            arr = arr[:, :msg.width * ch].reshape(msg.height, msg.width, ch)[::2, ::2]
        except Exception:
            return
        if enc.startswith('bgr'):
            b, g, r = arr[..., 0], arr[..., 1], arr[..., 2]
        else:
            r, g, b = arr[..., 0], arr[..., 1], arr[..., 2]
        mask = (r > RED_R_MIN) & (g < RED_G_MAX) & (b < RED_B_MAX)
        if int(mask.sum()) < RED_MIN_PIXELS:
            return
        ys, xs = np.nonzero(mask)
        u = float(xs.mean()) * 2
        v = float(ys.mean()) * 2
        cx, cy = msg.width / 2, msg.height / 2
        h = max(self.z - self.z0, 0.3)
        fx = msg.width / (2 * math.tan(CAM_HFOV / 2))
        fy = fx
        dx_right = (u - cx) / fx * h
        dy_forward = (cy - v) / fy * h
        fwd = dy_forward
        left = -dx_right

        now = time.monotonic()
        self.red_buffer.append((now, fwd, left))
        self.red_buffer = [(t, f, l) for (t, f, l) in self.red_buffer
                           if now - t < RED_BUFFER_TIMEOUT]
        self.red_buffer = self.red_buffer[-RED_BUFFER_SIZE:]
        fwds = [f for _, f, _ in self.red_buffer]
        lefts = [l for _, _, l in self.red_buffer]
        self.red = (float(np.median(fwds)), float(np.median(lefts)))
        self.t_red = now

    # ---------------- УТИЛИТЫ ----------------
    def log(self, text, period=1.0):
        now = time.monotonic()
        if now - self.last_log > period:
            self.get_logger().info(text)
            self.last_log = now

    def goto(self, state):
        self.get_logger().info(f'Состояние: {self.state} -> {state}')
        self.state = state
        self.t_state = time.monotonic()

    def request_mode(self, mode):
        if self.cli_mode.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = mode
            self.cli_mode.call_async(req)

    def request_arm(self):
        if FORCE_ARM and self.cli_cmd.service_is_ready():
            req = CommandLong.Request()
            req.command = 400
            req.param1 = 1.0
            req.param2 = 21196.0
            self.cli_cmd.call_async(req)
            return
        if self.cli_arm.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            self.cli_arm.call_async(req)

    def pose_fresh(self):
        return self.pose_ok and (time.monotonic() - self.t_pose) < POSE_TIMEOUT

    def pub_position(self, x, y, z, yaw):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.pose.orientation.z = math.sin(yaw / 2)
        m.pose.orientation.w = math.cos(yaw / 2)
        self.pub_pos.publish(m)

    def front_clearance(self):
        if self.scan is None:
            return float('inf')
        ang, r = self.scan
        sel = (np.abs((ang + np.pi) % (2 * np.pi) - np.pi) < math.radians(30))
        rr = r[sel]
        rr = rr[np.isfinite(rr)]
        return float(rr.min()) if rr.size else float('inf')

    def safety_check(self):
        if not self.pose_fresh():
            return 'pose устарела'
        if (self.z - self.z0) > TAKEOFF_ALT + ALT_CEILING_MARGIN:
            return f'высота превышена: {(self.z - self.z0):.2f} м'
        if self.mav is None or not self.mav.connected:
            return 'потеряна связь с FCU'
        return None

    def move_internal_target(self, dt):
        dx = self.target_x - self.tx
        dy = self.target_y - self.ty
        dist = math.hypot(dx, dy)
        if dist < 0.001:
            return
        step = min(APPROACH_SPEED * dt, dist)
        self.tx += dx / dist * step
        self.ty += dy / dist * step

    # ---------------- ГЛАВНЫЙ ЦИКЛ ----------------
    def tick(self):
        now = time.monotonic()
        S = self.state
        dt = 0.02

        if S == 'INIT':
            ready = (self.mav is not None and self.mav.connected and self.pose_fresh()
                     and self.pose_count >= 40 and self.scan is not None
                     and self.cli_arm.service_is_ready() and self.cli_mode.service_is_ready())
            self.log('Жду MAVROS, local_position, лидар, сервисы...')
            if not ready:
                return
            self.home = (self.x, self.y)
            self.z0 = self.z
            self.yaw0 = self.yaw
            self.tx = self.x
            self.ty = self.y
            self.target_x = self.x
            self.target_y = self.y
            self.get_logger().info(
                f'Старт: x={self.x:.2f} y={self.y:.2f} z={self.z:.2f} '
                f'yaw={math.degrees(self.yaw):.0f}°')
            self.z_sp = self.z0
            self.z_ref = self.z0 + TAKEOFF_ALT
            self.goto('PRESTREAM')
            return

        if S == 'PRESTREAM':
            self.pub_position(self.home[0], self.home[1], self.z0, self.yaw0)
            self.prestream_n += 1
            if self.prestream_n > 100:
                self.goto('ARMING')
            return

        if S in ('ARMING', 'TAKEOFF', 'SEARCH', 'APPROACH',
                 'SETTLE', 'HOVER', 'RETURN'):
            if S != 'ARMING':
                reason = self.safety_check()
                if reason:
                    self.get_logger().error(f'АВАРИЯ: {reason}')
                    self.goto('LANDING')
                    self.request_mode('AUTO.LAND')
                    return

        if S == 'ARMING':
            self.pub_position(self.home[0], self.home[1], self.z0, self.yaw0)
            if self.mav.mode == 'OFFBOARD' and self.mav.armed:
                self.goto('TAKEOFF')
                return
            self.log(f'ARMING: mode={self.mav.mode} armed={self.mav.armed}')
            if now - self.t_last_req > 2.0:
                self.t_last_req = now
                if self.mav.mode != 'OFFBOARD':
                    self.request_mode('OFFBOARD')
                if not self.mav.armed:
                    self.request_arm()
            if now - self.t_state > 60.0:
                self.get_logger().error('Не удалось армить за 60 с')
                self.goto('DONE')
            return

        if S == 'TAKEOFF':
            if self.z < self.z_ref + 0.1:
                self.z_sp = min(self.z_sp + TAKEOFF_RATE * dt, self.z_ref)
            self.pub_position(self.home[0], self.home[1], self.z_sp, self.yaw0)
            self.log(f'Взлёт: z={self.z - self.z0:.2f} м, '
                     f'уставка={self.z_sp - self.z0:.2f} м')
            if abs(self.z - self.z_ref) < 0.15 and self.z_sp >= self.z_ref:
                self.get_logger().info('Высота достигнута, начинаю поиск')
                self.tx = self.x
                self.ty = self.y
                self.red_buffer.clear()
                self.goto('SEARCH')
            if now - self.t_state > 60.0:
                self.get_logger().error('Взлёт не завершён за 60 с')
                self.goto('LANDING')
                self.request_mode('AUTO.LAND')
            return

        if S == 'SEARCH':
            if self.red is not None and len(self.red_buffer) >= RED_LOCK_DETECTIONS:
                f, l = self.red
                c, s = math.cos(self.yaw), math.sin(self.yaw)
                self.target_x = self.x + f * c - l * s
                self.target_y = self.y + f * s + l * c
                self.tx = self.x
                self.ty = self.y
                self.get_logger().info(
                    f'Красный найден: вперёд {f:+.2f}, влево {l:+.2f} → '
                    f'цель ({self.target_x:.2f}, {self.target_y:.2f})')
                self.red_buffer.clear()
                self.red = None
                self.goto('APPROACH')
                return

            clear = self.front_clearance()
            if clear < WALL_STOP_DIST:
                self.get_logger().warn(f'Стена впереди ({clear:.2f} м), стоп поиск')
                self.goto('RETURN')
                return
            if now - self.t_state > MAX_SEARCH_TIME:
                self.get_logger().warn(f'Поиск >{MAX_SEARCH_TIME:.0f} с, стоп')
                self.goto('RETURN')
                return

            self.tx += CRUISE_SPEED * dt * math.cos(self.yaw0)
            self.ty += CRUISE_SPEED * dt * math.sin(self.yaw0)
            self.target_x = self.tx
            self.target_y = self.ty
            self.pub_position(self.tx, self.ty, self.z_ref, self.yaw0)

            dist = math.hypot(self.x - self.home[0], self.y - self.home[1])
            self.log(f'Поиск: пройдено {dist:.1f} м, впереди {clear:.1f} м, '
                     f'высота {self.z - self.z0:.2f} м')
            return

        if S == 'APPROACH':
            self.move_internal_target(dt)
            self.pub_position(self.tx, self.ty, self.z_ref, self.yaw0)

            real_dist = math.hypot(self.target_x - self.x, self.target_y - self.y)
            self.log(f'Подлёт: до цели {real_dist:.2f} м, '
                     f'скорость {APPROACH_SPEED:.2f} м/с')

            if real_dist < DRONE_POS_TOL:
                self.get_logger().info(
                    f'Долетели до цели (осталось {real_dist:.2f} м), стабилизируюсь')
                self.goto('SETTLE')
                return

            if now - self.t_state > APPROACH_TIMEOUT:
                self.get_logger().warn('Подлёт затянулся, стабилизируюсь')
                self.goto('SETTLE')
            return

        if S == 'SETTLE':
            self.move_internal_target(dt)
            self.pub_position(self.tx, self.ty, self.z_ref, self.yaw0)
            left = SETTLE_TIME - (now - self.t_state)
            self.log(f'Стабилизация: осталось {left:.1f} с')
            if left <= 0:
                self.get_logger().info('Стабилизация завершена, зависаю')
                self.goto('HOVER')
            return

        if S == 'HOVER':
            self.pub_position(self.tx, self.ty, self.z_ref, self.yaw0)
            left = HOVER_TIME - (now - self.t_state)
            self.log(f'Зависание: осталось {left:.0f} с, '
                     f'высота {self.z - self.z0:.2f} м', period=2.0)
            if left <= 0:
                self.get_logger().info('Зависание завершено, возврат домой')
                self.target_x = self.home[0]
                self.target_y = self.home[1]
                self.goto('RETURN')
            return

        if S == 'RETURN':
            dx = self.home[0] - self.tx
            dy = self.home[1] - self.ty
            dist = math.hypot(dx, dy)

            if dist < HOME_TOL:
                self.get_logger().info('Вернулись на старт, посадка')
                self.goto('LANDING')
                self.request_mode('AUTO.LAND')
                return

            # Замедление у цели: чем ближе, тем медленнее
            if dist < LAND_APPROACH_DIST:
                speed = max(0.05, 0.3 * dist)
            else:
                speed = min(RETURN_SPEED, 0.5 * dist + 0.05)

            step = speed * dt
            if step > dist:
                step = dist
            self.tx += dx / dist * step
            self.ty += dy / dist * step
            self.pub_position(self.tx, self.ty, self.z_ref, self.yaw0)
            self.log(f'Возврат: осталось {dist:.2f} м, скорость {speed:.2f} м/с')
            return

        if S == 'LANDING':
            if self.mav and self.mav.mode != 'AUTO.LAND' and now - self.t_last_req > 1.0:
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
    node = SearchMission()
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
