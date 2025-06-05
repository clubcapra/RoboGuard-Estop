# estop_manager.py
# This node triggers a GPIO estop line on Raspberry Pi (or equivalent)
# based on /estop_general_h heartbeat status and /estop_manual_s signal.

import lgpio

class EstopManager(Node):
    def __init__(self):
        super().__init__('estop_manager')

        self.timeout_ms = self.declare_parameter('timeout_ms', 200).get_parameter_value().integer_value
        self.heartbeat_freq = self.declare_parameter('heartbeat_freq', 10).get_parameter_value().integer_value
        self.gpio_pin = self.declare_parameter('gpio_pin', 26).get_parameter_value().integer_value

        self.last_general = self.get_clock().now()
        self.general_val = False
        self.manual_override = None

        self.create_subscription(Bool, '/estop_general_h', self.cb_general, 10)
        self.create_subscription(Bool, '/estop_manual_s', self.cb_manual, 10)

        self.h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.h, self.gpio_pin)

        self.gpio_state = None
        self.set_gpio(True)  # default to estop ON

        self.timer = self.create_timer(1.0 / self.heartbeat_freq, self.check)

    def cb_general(self, msg):
        self.general_val = msg.data
        self.last_general = self.get_clock().now()

    def cb_manual(self, msg):
        if msg.data:
            self.set_gpio(True)
            self.manual_override = True
        elif not msg.data and not self.general_val:
            self.set_gpio(False)
            self.manual_override = False

    def check(self):
        now = self.get_clock().now()
        timeout = (now - self.last_general).nanoseconds / 1e6 > self.timeout_ms

        if not self.manual_override:
            self.set_gpio(self.general_val or timeout)

    def set_gpio(self, engage):
        value = 1 if engage else 0
        if self.gpio_state != value:
            lgpio.gpio_write(self.h, self.gpio_pin, value)
            self.gpio_state = value
            self.get_logger().info(f"GPIO {self.gpio_pin} set to {'HIGH (TRIGGERED)' if engage else 'LOW (OK)'}")

    def __del__(self):
        lgpio.gpiochip_close(self.h)