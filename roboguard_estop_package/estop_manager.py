import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import lgpio

class EstopManager(Node):
    def __init__(self):
        super().__init__('estop_manager')

        self.timeout_ms = 150
        self.pin = 29  # GPIO pin to control estop circuit
        self.gpio_state = None  # Track last GPIO state
        self.last_valid_msg_time = self.get_clock().now()
        self.general_estop_topic = '/estop_general_h'

        # Setup GPIO
        self.h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.h, self.pin)
        self.set_gpio_state(False)  # Default to OK state on startup (LOW)

        # Subscribe to estop heartbeat
        self.create_subscription(Bool, self.general_estop_topic, self.estop_callback, 10)

        # Start timer to check for heartbeat timeout (50ms is reasonable for a 150ms timeout)
        self.timer = self.create_timer(0.05, self.check_timeout)  # 10ms

    def estop_callback(self, msg):
        try:
            if isinstance(msg.data, bool):
                self.last_valid_msg_time = self.get_clock().now()
                if msg.data:  # true means estop is triggered
                    self.set_gpio_state(True)
                else:
                    self.set_gpio_state(False)
            else:
                self.get_logger().warn("Non-boolean message received, ignoring")
                # no state change; timeout will handle failure
        except Exception as e:
            self.get_logger().error(f"Exception in estop_callback: {e}")
            # ignore and let timeout trigger estop

    def check_timeout(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_valid_msg_time).nanoseconds / 1e6  # in ms
        if elapsed > self.timeout_ms:
            self.set_gpio_state(True)  # Timeout: trigger estop

    def set_gpio_state(self, triggered):
        value = 1 if triggered else 0
        if self.gpio_state != value:
            lgpio.gpio_write(self.h, self.pin, value)
            self.gpio_state = value
            state_str = "HIGH (TRIGGERED)" if value else "LOW (OK)"
            self.get_logger().info(f"GPIO {self.pin} set to {state_str}")

    def cleanup(self):
        self.set_gpio_state(False)  # Set GPIO LOW on cleanup
        lgpio.gpiochip_close(self.h)

    def __del__(self):
        self.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = EstopManager()
    try:
        rclpy.spin(node)
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()