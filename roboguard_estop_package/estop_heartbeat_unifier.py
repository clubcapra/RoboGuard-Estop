estop_heartbeat_unifier.py
# This node listens to controller, UI, and hardware heartbeat estops.
# It outputs a unified heartbeat signal: True if ANY estop heartbeat is triggered or times out.

class EstopHeartbeatUnifier(Node):
    def __init__(self):
        super().__init__('estop_heartbeat_unifier')

        self.timeout_ms = self.declare_parameter('timeout_ms', 200).get_parameter_value().integer_value
        self.heartbeat_freq = self.declare_parameter('heartbeat_freq', 10).get_parameter_value().integer_value

        self.last_ctrl = self.get_clock().now()
        self.last_ui = self.get_clock().now()
        self.last_hw = self.get_clock().now()

        self.ctrl_val = False
        self.ui_val = False
        self.hw_val = False

        self.create_subscription(Bool, '/estop_controller_h', self.cb_ctrl, 10)
        self.create_subscription(Bool, '/estop_ui_h', self.cb_ui, 10)
        self.create_subscription(Bool, '/estop_hardware_h', self.cb_hw, 10)

        self.pub = self.create_publisher(Bool, '/estop_general_h', 10)
        self.timer = self.create_timer(1.0 / self.heartbeat_freq, self.check)

    def cb_ctrl(self, msg):
        self.ctrl_val = msg.data
        self.last_ctrl = self.get_clock().now()

    def cb_ui(self, msg):
        self.ui_val = msg.data
        self.last_ui = self.get_clock().now()

    def cb_hw(self, msg):
        self.hw_val = msg.data
        self.last_hw = self.get_clock().now()

    def check(self):
        now = self.get_clock().now()

        timeout_ctrl = (now - self.last_ctrl).nanoseconds / 1e6 > self.timeout_ms
        timeout_ui = (now - self.last_ui).nanoseconds / 1e6 > self.timeout_ms
        timeout_hw = (now - self.last_hw).nanoseconds / 1e6 > self.timeout_ms

        triggered = timeout_ctrl or timeout_ui or timeout_hw or self.ctrl_val or self.ui_val or self.hw_val
        self.pub.publish(Bool(data=triggered))