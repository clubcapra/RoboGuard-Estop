import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time
import lgpio

# This node monitors a manual estop signal on the /manual_estop topic
# The E-Stop is triggered when a positive message is received on the topic.
# To trigger the E-Stop, the GPIO pin is set to LOW since the circuit is normally closed.
# Expected message: {data: true} to enable E-Stop, {data: false} to disable E-Stop.

# Sources for the manual E-Stop (just for info, not the responsibility of this node):
# - Manual trigger from controller heartbeat monitor node
# - Manual trigger from UI heartbeat monitor node
# - If the STM32 calls it's own E-Stop 
# - If the emergency mushroom button is pressed


class EstopMonitor(Node):
    def __init__(self):
        super().__init__('main_estop_trigger')
        self.sub = self.create_subscription(Bool, '/manual_estop', self.estopsignal_callback)
        self.pin = 29 # E-Stop pin
        self.gpio_state = True  # Track current GPIO state

        self.h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.h, self.pin)

        # Set GPIO HIGH on startup since no estop has been received yet
        self.set_gpio_state(1)

    def estopsignal_callback(self, msg):
        # $self.get_logger().info("manual E-Stop called: " + str(msg.data))
        self.set_gpio_state(msg.data)

    def set_gpio_state(self, data):
        # If the message is false, set GPIO to LOW (disable E-Stop)
        # If the message is true, null, not a boolean, or throws, set GPIO to HIGH (enable E-Stop)
        try:
            if data is None or not isinstance(data, bool):
                self.get_logger().warn("Invalid estop message received, triggering E-Stop")
                value = 1
            else:
                value = 0 if data else 1
            # Only set GPIO state if it has changed
        except Exception as e:
            self.get_logger().error(f"Error in estopsignal callback: {e}")
            value = 1  # Default to HIGH on error



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
    self.get_logger().info("Deprecated node, to delete")
    #rclpy.init(args=args)
    #node = EstopMonitor()
    #try:
    #    rclpy.spin(node)
    #finally:
    #    node.cleanup()
    #    node.destroy_node()
    #    rclpy.shutdown()