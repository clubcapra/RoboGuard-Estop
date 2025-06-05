# estop_hardware_monitor.py
# This node listens to the messages on topics /estop_stm32_status and /estop_mushroom_status.
# It expects data:true if the STM32 or mushroom estop is active, and data:false if inactive.
# These should be published by roboguard_micro_ros.cpp every 66ms (subject to change).

# If no message is received within the timeout period, we will trigger the estop for fear that the node has crashed or the hardware is malfunctioning.

# It does not validate the estop status (true/false); it only cares that a signal is received.
# This is because if a message is published, it means the hardware is functioning correctly and should engage it's own estop.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class EstopHardwareMonitor(Node):
    def __init__(self):
        super().__init__('estop_hardware_monitor')

        self.timeout_ms = 400  # Acceptable timeout since the heartbeats are published at 15.15Hz

        self.stm32_status_topic = '/estop_stm32_status' # Topic where the status of the STM32 estop is published
        self.last_stm32_status = False

        self.mushroom_status_topic = '/estop_mushroom_status' # Topic where the status of the mushroom estop is published
        self.last_mushroom_status = False

        self.hadware_estop_topic = '/estop_hardware_h' # Topic where this node will publish the general hardware estop heartbeat
        self.timer_period = 0.1 # Timer period to publish heartbeat (0.1 = 10Hz, same as the controller and UI heartbeat)

        self.stm32_sub = self.create_subscription(Bool, self.stm32_status_topic, self.stm32_callback)
        self.mushroom_sub = self.create_subscription(Bool, self.mushroom_status_topic, self.mushroom_callback)
        self.publisher_ = self.create_publisher(Bool, self.hadware_estop_topic, 10)


        self.last_stm32_time = self.get_clock().now()
        self.last_mushroom_time = self.get_clock().now()

        self.create_subscription(Bool, '/estop_stm32_status', self.stm32_cb, 10)
        self.create_subscription(Bool, '/estop_mushroom_status', self.mushroom_cb, 10)
        self.pub = self.create_publisher(Bool, '/estop_hardware_h', 10)

        timer_period = 1.0 / self.heartbeat_freq
        self.timer = self.create_timer(timer_period, self.check_timeout)

    def stm32_cb(self, msg):
        self.last_stm32_time = self.get_clock().now()

    def mushroom_cb(self, msg):
        self.last_mushroom_time = self.get_clock().now()

    def check_timeout(self):
        now = self.get_clock().now()
        delta_stm32 = (now - self.last_stm32_time).nanoseconds / 1e6
        delta_mushroom = (now - self.last_mushroom_time).nanoseconds / 1e6

        out = Bool()
        out.data = delta_stm32 > self.timeout_ms or delta_mushroom > self.timeout_ms
        self.pub.publish(out)