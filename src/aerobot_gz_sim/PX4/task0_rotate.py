#!/usr/bin/env python3
"""
Миссия: взлёт на 1.5 м, полный оборот вокруг оси, посадка.
Использует MAVROS (namespace /uav1).
Запуск: python3 task0_rotate.py
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode

NS = '/uav1'
TAKEOFF_ALT = 1.5          # высота взлёта (м)
ROTATE_SPEED = 0.5         # скорость вращения (рад/с)
TAKEOFF_RATE = 0.4         # скорость взлёта (м/с)
POSE_TIMEOUT = 2.0         # таймаут свежести позы (с)
FORCE_ARM = True           # принудительный арминг (обход preflight)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class RotateMission(Node):
    def __init__(self):
        super().__init__('rotate_mission')

        self.mav = None
        self.pose_ok = False
        self.pose_count = 0
        self.t_pose = 0.0
        self.x = self.y = self.z = self.yaw = 0.0

        qos = qos_profile_sensor_data
        self.create_subscription(State, f'{NS}/mavros/state', self.cb_state, qos)
        self.create_subscription(
            PoseStamped, f'{NS}/mavros/local_position/pose', self.cb_pose, qos)

        self.pub_pos = self.create_publisher(
            PoseStamped, f'{NS}/mavros/setpoint_position/local', 10)
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
        self.yaw_sp = 0.0
        self.t_last_req = 0.0
        self.prestream_n = 0
        self.last_log = 0.0

        self.create_timer(0.05, self.tick)  # 20 Гц

    # ---------------- колбэки ----------------
    def cb_state(self, msg):
        self.mav = msg

    def cb_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        vals = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if any(not math.isfinite(v) for v in vals):
            self.pose_ok = False
            return
        self.x, self.y, self.z = p.x, p.y, p.z
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))
        self.t_pose = time.monotonic()
        self.pose_ok = True
        self.pose_count += 1

    # ---------------- утилиты ----------------
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
            req.command = 400        # MAV_CMD_COMPONENT_ARM_DISARM
            req.param1 = 1.0         # arm
            req.param2 = 21196.0     # force
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

    # ---------------- главный цикл ----------------
    def tick(self):
        now = time.monotonic()
        S = self.state

        if S == 'INIT':
            ready = (self.mav is not None and self.mav.connected
                     and self.pose_fresh() and self.pose_count >= 40
                     and self.cli_arm.service_is_ready()
                     and self.cli_mode.service_is_ready())
            self.log('Жду MAVROS, local_position, сервисы...')
            if not ready:
                return
            self.home = (self.x, self.y)
            self.z0 = self.z
            self.yaw0 = self.yaw
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
            if self.prestream_n > 40:
                self.goto('ARMING')
            return

        # Защита: pose устарела (кроме первых 3 с после смены режима)
        if S in ('ARMING', 'TAKEOFF', 'ROTATE'):
            if (now - self.t_state) > 3.0 and not self.pose_fresh():
                self.get_logger().error('АВАРИЯ: pose устарела')
                self.goto('LANDING')
                self.request_mode('AUTO.LAND')
                return
            if self.mav and self.mav.mode != 'OFFBOARD' and S != 'ARMING':
                self.get_logger().warn(f'Режим сменён на {self.mav.mode}')
                self.goto('DONE')
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
            self.log(f'Взлёт: z={self.z - self.z0:.2f} м, '
                     f'уставка={self.z_sp - self.z0:.2f} м')
            if abs(self.z - self.z_ref) < 0.15 and self.z_sp >= self.z_ref:
                self.get_logger().info('Высота 1.5 м достигнута, начинаю оборот')
                self.yaw_sp = self.yaw0
                self.goto('ROTATE')
            if now - self.t_state > 60.0:
                self.get_logger().error('Взлёт не завершён за 60 с')
                self.goto('LANDING')
                self.request_mode('AUTO.LAND')
            return

        if S == 'ROTATE':
            dt = 0.05
            self.yaw_sp += ROTATE_SPEED * dt
            self.pub_position(self.home[0], self.home[1], self.z_ref, self.yaw_sp)
            commanded = self.yaw_sp - self.yaw0
            prog = commanded / (2 * math.pi) * 100
            self.log(f'Оборот: {math.degrees(commanded):.0f}° / 360° ({prog:.0f}%)')
            if commanded >= 2 * math.pi:
                self.get_logger().info('Оборот завершён')
                self.goto('LANDING')
                self.request_mode('AUTO.LAND')
            if now - self.t_state > 30.0:
                self.get_logger().warn('Оборот длится >30 с, сажусь')
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


def main():
    rclpy.init()
    node = RotateMission()
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
