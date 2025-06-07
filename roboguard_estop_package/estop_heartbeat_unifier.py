import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class EstopHeartbeatUnifier(Node):
    def __init__(self):
        super().__init__('estop_heartbeat_unifier')

        self.timeout_ms = 150  # Max time between valid messages
        self.timer_period = 0.01  # 10 ms publishing frequency
        self.output_topic = '/estop_general_h'

        self.topics = {
            '/estop_controller_h': {'last_value': None, 'last_time': self.get_clock().now()},
            '/estop_ui_h':         {'last_value': None, 'last_time': self.get_clock().now()},
            '/estop_hardware_h':   {'last_value': None, 'last_time': self.get_clock().now()}
        }

        # Subscriptions
        for topic in self.topics.keys():
            self.create_subscription(Bool, topic, self._make_callback(topic), 10)

        # Publisher
        self.pub = self.create_publisher(Bool, self.output_topic, 10)

        # Timer to check and publish
        self.timer = self.create_timer(self.timer_period, self.check_and_publish)

    def _make_callback(self, topic_name):
        def callback(msg):
            try:
                if isinstance(msg.data, bool):
                    self.topics[topic_name]['last_value'] = msg.data
                    self.topics[topic_name]['last_time'] = self.get_clock().now()
            except Exception as e:
                self.get_logger().warn(f"Exception in {topic_name} callback: {e}")
        return callback

    def check_and_publish(self):
        now = self.get_clock().now()
        estop_required = False

        for topic, state in self.topics.items():
            time_since_last = (now - state['last_time']).nanoseconds / 1e6  # ms
            if (
                time_since_last > self.timeout_ms or
                state['last_value'] is None or
                state['last_value'] != False
            ):
                estop_required = True
                break

        msg = Bool()
        msg.data = estop_required
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EstopHeartbeatUnifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()