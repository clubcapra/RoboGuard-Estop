import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import lgpio


class PowerSupplyToggler(Node):
    """
    This node monitors messages on the /powersupply_mode topic and toggles a GPIO pin accordingly.

    - If it receives `True`, it sets the GPIO pin HIGH (E-Stop TRIGGERED).
    - If it receives `False`, it sets the GPIO pin LOW (E-Stop OK).
    """

    def __init__(self):
        super().__init__('roboguard_powersupply_toggler')
        self.pin = 31  # GPIO pin used to control E-Stop state
        self.gpio_state = None  # Track current GPIO state

        # Open GPIO chip and claim output pin
        self.h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.h, self.pin)

        # Set default state to LOW (safe startup state)
        self.set_gpio_state(0)

        # Subscribe to the /powersupply_mode topic
        self.sub = self.create_subscription(
            Bool,
            '/powersupply_mode',
            self.powersupply_callback,
            10
        )

        self.get_logger().info("PowerSupplyToggler node started, listening on /powersupply_mode")

    def powersupply_callback(self, msg: Bool):
        value = 1 if msg.data else 0
        state_str = "TRIGGERED (HIGH)" if value == 1 else "OK (LOW)"
        self.get_logger().info(f"Received message: {msg.data} → Setting GPIO {self.pin} to {state_str}")
        self.set_gpio_state(value)

    def set_gpio_state(self, value):
        if self.gpio_state != value:
            lgpio.gpio_write(self.h, self.pin, value)
            self.gpio_state = value

    def cleanup(self):
        # Set to safe LOW state on shutdown
        self.set_gpio_state(0)
        lgpio.gpiochip_close(self.h)
        self.get_logger().info("Cleaned up GPIO and shutting down")

    def __del__(self):
        self.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = PowerSupplyToggler()
    try:
        rclpy.spin(node)
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()