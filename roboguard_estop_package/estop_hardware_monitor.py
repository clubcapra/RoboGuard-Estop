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

class EstopHardwareMonitor(Node):
    def __init__(self):
        super().__init__('estop_hardware_monitor')

        self.timeout_ms = 400  # Heartbeat timeout in milliseconds (should publish every 66ms)
        self.timer_period = 0.01  # 10ms = 100Hz publishing loop

        # Initialize last message timestamps
        self.last_stm32_time = None
        self.last_mushroom_time = None

        # Topic names
        self.stm32_status_topic = '/estop_stm32_status'
        self.mushroom_status_topic = '/estop_mushroom_status'
        self.output_topic = '/estop_hardware_h'

        # Subscriptions
        self.create_subscription(Bool, self.stm32_status_topic, self.stm32_callback, 10)
        self.create_subscription(Bool, self.mushroom_status_topic, self.mushroom_callback, 10)

        # Publisher
        self.pub = self.create_publisher(Bool, self.output_topic, 10)

        # Current estop state
        self.estop_ok = False  # Start conservative
        self.last_published_state = None  # To track transitions

        # Timer to check heartbeats and publish
        self.timer = self.create_timer(self.timer_period, self.check_timeout)

    def stm32_callback(self, msg):
        try:
            if isinstance(msg.data, bool):
                self.last_stm32_time = self.get_clock().now()
            else:
                self.get_logger().warn("Received non-boolean STM32 estop message.")
        except Exception as e:
            self.get_logger().error(f"Exception in STM32 callback: {e}")

    def mushroom_callback(self, msg):
        try:
            if isinstance(msg.data, bool):
                self.last_mushroom_time = self.get_clock().now()
            else:
                self.get_logger().warn("Received non-boolean mushroom estop message.")
        except Exception as e:
            self.get_logger().error(f"Exception in mushroom callback: {e}")

    def check_timeout(self):
        now = self.get_clock().now()
        try:
            # Check deltas
            stm32_valid = self.last_stm32_time is not None and \
                (now - self.last_stm32_time).nanoseconds / 1e6 <= self.timeout_ms
            mushroom_valid = self.last_mushroom_time is not None and \
                (now - self.last_mushroom_time).nanoseconds / 1e6 <= self.timeout_ms

            # Determine if we're OK
            new_estop_ok = stm32_valid and mushroom_valid

            # Only log transitions (GOOD → BAD, BAD → GOOD)
            if new_estop_ok != self.last_published_state:
                if new_estop_ok:
                    self.get_logger().info("E-STOP: Hardware heartbeat back up")
                else:
                    self.get_logger().warn("E-STOP: Hardware heartbeat lost")

                self.last_published_state = new_estop_ok

            # Publish result
            msg = Bool()
            # invert (false = estop is NOT ACTIVE, true = estop is ACTIVE)
            msg.data = not new_estop_ok # hardcode this as true if both heartbeats are not implemented
            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Exception in heartbeat check: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EstopHardwareMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()