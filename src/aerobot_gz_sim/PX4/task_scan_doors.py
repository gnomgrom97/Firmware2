#!/usr/bin/env python3
"""
Миссия: взлёт на 1.5 м, оборот 360° с определением дверных проёмов через лидар.
Адаптивная детекция: проём = сектор, где дистанция резко больше медианы стен.
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode

NS = '/uav1'
TAKEOFF_ALT = 1.5
TAKEOFF_RATE = 0.4
POSE_TIMEOUT = 2.0
FORCE_ARM = True

WALL_DIST_MAX = 4.0
DOOR_MIN_DIST = 4.5
DOOR_MIN_WIDTH_DEG = 8
DOOR_RATIO = 1.6
MIN_VALID_RANGE = 0.2
SCAN_DURATION = 1.0        # сколько секунд собирать снимки


class ScanDoorsMission(Node):
    def __init__(self):
        super().__init__('scan_doors_mission')
        self.mav = None
        self.pose_ok = False
        self.pose_count = 0
        self.t_pose = 0.0
        self.x = self.y = self.z = self.yaw = 0.0
        self.scan = None
        self.t_scan = 0.0
        self.samples = []

        qos = qos_profile_sensor_data
        self.create_subscription(State, f'{NS}/mavros/state', self.cb_state, qos)
        self.create_subscription(PoseStamped, f'{NS}/mavros/local_position/pose', self.cb_pose, qos)
        self.create_subscription(LaserScan, f'{NS}/scan', self.cb_scan, qos)
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
        self.scan_start_t = 0.0
        self.t_last_req = 0.0
        self.prestream_n = 0
        self.last_log = 0.0
        self.create_timer(0.05, self.tick)

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
        lo = max(msg.range_min, MIN_VALID_RANGE)
        valid = np.isfinite(r) & (r >= lo)
        r = np.where(valid, r, np.inf)
        self.scan = (ang, r)
        self.t_scan = time.monotonic()

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

    def analyze_scan_for_doors(self):
        if self.scan is None:
            return []
        ang, r = self.scan
        n = len(r)

        finite = np.isfinite(r) & (r < 30.0)
        if finite.sum() < 10:
            return []
        median_r = float(np.median(r[finite]))

        threshold = max(DOOR_MIN_DIST, median_r * DOOR_RATIO)

        door_mask = (r > threshold)

        doors = []
        i = 0
        while i < n:
            if door_mask[i]:
                j = i
                while j < n and door_mask[j]:
                    j += 1
                width_rad = (j - i) * (ang[1] - ang[0]) if n > 1 else 0
                width_deg = math.degrees(width_rad)
                if width_deg >= DOOR_MIN_WIDTH_DEG:
                    center_ang = float((ang[i] + ang[j - 1]) / 2)
                    sector = r[i:j]
                    sector_finite = sector[np.isfinite(sector)]
                    dist = float(np.median(sector_finite)) if sector_finite.size else float('inf')
                    doors.append((center_ang, width_deg, dist, threshold))
                i = j
            else:
                i += 1
        return doors

    def tick(self):
        now = time.monotonic()
        S = self.state

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
            self.get_logger().info(
                f'Старт: x={self.x:.2f} y={self.y:.2f} z={self.z:.2f} yaw={math.degrees(self.yaw):.0f}°')
            self.z_sp = self.z0
            self.z_ref = self.z0 + TAKEOFF_ALT
            self.goto('PRESTREAM')
            return

        if S == 'PRESTREAM':
            self.pub_position(self.home[0], self.home[1], self.z0, self.yaw0)
            self.prestream_n += 1
            if self.prestream_n > 40:
                self.goto('ARMING')
            return

        if S in ('ARMING', 'TAKEOFF', 'SCAN'):
            if (now - self.t_state) > 3.0 and not self.pose_fresh():
                self.get_logger().error('АВАРИЯ: pose устарела')
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
            self.z_sp = min(self.z_sp + TAKEOFF_RATE * 0.05, self.z_ref)
            self.pub_position(self.home[0], self.home[1], self.z_sp, self.yaw0)
            self.log(f'Взлёт: z={self.z - self.z0:.2f} м, уставка={self.z_sp - self.z0:.2f} м')
            if abs(self.z - self.z_ref) < 0.15 and self.z_sp >= self.z_ref:
                self.get_logger().info('Высота достигнута, начинаю скан')
                self.samples = []
                self.scan_start_t = now
                self.goto('SCAN')
            if now - self.t_state > 60.0:
                self.get_logger().error('Взлёт не завершён за 60 с')
                self.goto('LANDING')
                self.request_mode('AUTO.LAND')
            return

        if S == 'SCAN':
            self.pub_position(self.home[0], self.home[1], self.z_ref, self.yaw0)

            if self.scan is not None and (now - self.t_scan) < 0.5:
                ang, r = self.scan
                self.samples.append((self.yaw, ang.copy(), r.copy()))

            left = SCAN_DURATION - (now - self.scan_start_t)
            self.log(f'Скан: собрано {len(self.samples)} снимков, осталось {left:.1f} с')

            if now - self.scan_start_t >= SCAN_DURATION:
                self.get_logger().info('Скан завершён, анализирую проёмы...')
                self.report_doors()
                self.goto('LANDING')
                self.request_mode('AUTO.LAND')
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

    def report_doors(self):
        if not self.samples:
            self.get_logger().warn('Нет снимков лидара для анализа')
            return

        all_doors = []
        for yaw, ang, r in self.samples:
            saved = self.scan
            self.scan = (ang, r)
            doors = self.analyze_scan_for_doors()
            self.scan = saved
            for door in doors:
                center_ang, width_deg, dist = door[0], door[1], door[2]
                world_angle = math.atan2(math.sin(yaw + center_ang), math.cos(yaw + center_ang))
                all_doors.append((world_angle, width_deg, dist))

        if not all_doors:
            self.get_logger().info('Дверных проёмов не обнаружено.')
            return

        all_doors.sort()
        groups = []
        for a, w, d in all_doors:
            if groups and abs(groups[-1][0][0] - a) < math.radians(15):
                groups[-1].append((a, w, d))
            else:
                groups.append([(a, w, d)])

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'ОБНАРУЖЕНО ДВЕРНЫХ ПРОЁМОВ: {len(groups)}')
        self.get_logger().info('=' * 60)
        for i, grp in enumerate(groups, 1):
            avg_a = sum(x[0] for x in grp) / len(grp)
            max_w = max(x[1] for x in grp)
            avg_d = sum(x[2] for x in grp) / len(grp)
            self.get_logger().info(
                f'Проём {i}: угол {math.degrees(avg_a):+.0f}° от старта, '
                f'ширина ~{max_w:.0f}°, дистанция ~{avg_d:.1f} м')
        self.get_logger().info('=' * 60)


def main():
    rclpy.init()
    node = ScanDoorsMission()
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
