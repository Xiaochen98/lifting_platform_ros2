#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import select
import tty
import termios

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


INSTR = """
Lift Teleop (方向键控制)：
  ↑  持续上升   (publish "up")
  ↓  持续下降   (publish "down")
  空格/ s 停止  (publish "stop")
  q  退出

提示：按住↑/↓会以设定频率持续发送指令；松开后自动发送 stop。
"""

class LiftTeleopKey(Node):
    def __init__(self):
        super().__init__('lift_teleop_key')

        # 发布者：/lift/cmd (std_msgs/String)
        self.cmd_pub = self.create_publisher(String, '/lift/cmd', 10)

        # 参数：重复发送频率（Hz），以及空闲多久自动发送stop
        self.declare_parameter('repeat_rate', 10.0)       # 发送频率
        self.declare_parameter('idle_stop_timeout', 0.2)  # 松手后多快发送stop（秒）

        self.repeat_rate = float(self.get_parameter('repeat_rate').value)
        self.idle_stop_timeout = float(self.get_parameter('idle_stop_timeout').value)

        self.last_cmd = 'stop'
        self.last_press_time = 0.0
        self.key_held_cmd = None  # 'up' / 'down' / None

        # 终端设置为原始模式，便于读取方向键
        self.fd = sys.stdin.fileno()
        self.original_term_attr = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

        self.get_logger().info(INSTR)

        # 定时器循环：读取按键 + 重复发布
        self.timer = self.create_timer(1.0 / max(self.repeat_rate, 1.0), self.loop)

    def destroy_node(self):
        # 恢复终端
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original_term_attr)
        except Exception:
            pass
        return super().destroy_node()

    def loop(self):
        # 非阻塞读取键盘
        key = self._get_key_nonblocking()
        now = time.time()

        if key is not None:
            # 解析按键
            if key == '\x1b':  # 可能是方向键的起始 ESC
                seq = self._read_arrow_sequence_nonblocking()
                if seq == '[A':          # ↑
                    self.key_held_cmd = 'up'
                    self.last_press_time = now
                    self._publish('up')
                elif seq == '[B':        # ↓
                    self.key_held_cmd = 'down'
                    self.last_press_time = now
                    self._publish('down')
                else:
                    # 其他方向键/未知序列，忽略
                    pass

            elif key in (' ', 's', 'S'):
                self.key_held_cmd = None
                self._publish('stop')

            elif key in ('q', 'Q'):
                self._publish('stop')
                self.get_logger().info('退出 teleop')
                rclpy.shutdown()
                return
            else:
                # 其他键忽略
                pass

        # 如果按住↑/↓，按设定频率持续发送
        if self.key_held_cmd in ('up', 'down'):
            self._publish(self.key_held_cmd)
            self.last_press_time = now

        # 若一段时间没有任何按键，自动发送 stop（防抖/安全）
        if (self.key_held_cmd is None) and (now - self.last_press_time > self.idle_stop_timeout):
            if self.last_cmd != 'stop':
                self._publish('stop')

    def _publish(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.last_cmd = cmd

    def _get_key_nonblocking(self):
        # 使用 select 非阻塞读取单字符
        dr, _, _ = select.select([sys.stdin], [], [], 0.0)
        if dr:
            c = sys.stdin.read(1)
            return c
        return None

    def _read_arrow_sequence_nonblocking(self):
        # 方向键一般是 ESC [ A/B/C/D
        # 这里尝试立刻读后续两个字符；若读不到就返回空
        seq = ''
        start = time.time()
        while len(seq) < 2 and (time.time() - start) < 0.005:
            dr, _, _ = select.select([sys.stdin], [], [], 0.0)
            if dr:
                seq += sys.stdin.read(1)
            else:
                break
        return seq


def main():
    rclpy.init()
    node = LiftTeleopKey()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
