#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rosbag2_interfaces.srv import SetRate, Stop

class BagSpeedManager(Node):
    def __init__(self):
        super().__init__('bag_speed_manager')

        # Parameters
        self.slow_rate = 0.1
        self.wait_duration = 5.0  # Seconds

        # State tracking
        self.button_is_pressed = False
        self.last_release_time = self.get_clock().now()
        self.current_rate = 1.0
        self.pending_stop = False

        # Subscriber
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Service Clients
        self.rate_client = self.create_client(SetRate, '/rosbag2_player/set_rate')
        self.stop_client = self.create_client(Stop, '/rosbag2_player/stop')

        # Timer to check for the timeout
        self.timer = self.create_timer(0.5, self.check_timeout)

    def joy_callback(self, msg):
        if len(msg.buttons) < 1:
            return

        is_currently_pressed = (msg.buttons[0] == 1)

        # 1. Transition: Pressed (Slow Down)
        if is_currently_pressed and not self.button_is_pressed:
            self.get_logger().info(f"Button pressed! Slowing to {self.slow_rate}")
            self.change_rate(self.slow_rate)
            self.pending_stop = False

        # 2. Transition: Released (Start Stop Countdown)
        elif not is_currently_pressed and self.button_is_pressed:
            self.get_logger().info(f"Button released. Stopping bag in {self.wait_duration}s...")
            self.last_release_time = self.get_clock().now()
            self.pending_stop = True

        self.button_is_pressed = is_currently_pressed

    def check_timeout(self):
        if self.pending_stop and not self.button_is_pressed:
            now = self.get_clock().now()
            elapsed = (now - self.last_release_time).nanoseconds / 1e9

            if elapsed >= self.wait_duration:
                self.get_logger().info("Timeout reached. Calling /rosbag2_player/stop")
                self.trigger_stop()
                self.pending_stop = False

    def change_rate(self, target):
        if not self.rate_client.service_is_ready():
            return
        req = SetRate.Request()
        req.rate = target
        self.rate_client.call_async(req)

    def trigger_stop(self):
        if not self.stop_client.service_is_ready():
            self.get_logger().error("Stop service not available!")
            return
        req = Stop.Request()
        self.stop_client.call_async(req)

def main():
    rclpy.init()
    node = BagSpeedManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
