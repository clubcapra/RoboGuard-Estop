import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time
import lgpio

# This node monitors the heartbeat of the system and triggers an E-Stop if no heartbeat is received within a specified timeout period.
# It uses GPIO pin 29 to control the E-Stop state.
# The heartbeat topic is expected to be published by another node in the system.
# The E-Stop is triggered (GPIO HIGH) if no heartbeat is received within the timeout period.
# The GPIO pin is set to LOW (OK) when a heartbeat is received.

class EstopMonitor(Node):
    def __init__(self):
        super().__init__('roboguard_estop_monitor')
        self.sub = self.create_subscription(Bool, '/heartbeat', self.heartbeat_callback, 10) # TODO: PUT THE CORRECT TOPIC NAME
        self.last_heartbeat_time = None  # Will be set on first heartbeat
        self.timeout = 0.15  # 150 ms, figure this should be enough
        self.pin = 29 # E-Stop pin
        self.gpio_state = None  # Track current GPIO state to avoid spamming output

        self.h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.h, self.pin)

        # Set GPIO HIGH on startup since no heartbeat has been received yet
        self.set_gpio_state(1)

        self.timer = self.create_timer(0.05, self.check_timeout)

    def heartbeat_callback(self, msg):
        self.last_heartbeat_time = time.monotonic()
        self.get_logger().info("Heartbeat received")

    def check_timeout(self):
        current_time = time.monotonic()
        if self.last_heartbeat_time is None or (current_time - self.last_heartbeat_time > self.timeout):
            self.set_gpio_state(1)  # Triggered
        else:
            self.set_gpio_state(0)  # Healthy

    def set_gpio_state(self, value):
        if self.gpio_state != value:
            lgpio.gpio_write(self.h, self.pin, value)
            state_str = "HIGH (TRIGGERED)" if value == 1 else "LOW (OK)"
            self.get_logger().info(f"GPIO {self.pin} set to {state_str}")
            self.gpio_state = value

    def cleanup(self):
        self.set_gpio_state(0)
        lgpio.gpiochip_close(self.h)

    def __del__(self):
        self.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = EstopMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
