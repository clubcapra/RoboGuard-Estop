import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
import time

class EstopMonitor(Node):
    def __init__(self):
        super().__init__('estop_monitor')
        self.sub = self.create_subscription(Bool, '/heartbeat', self.heartbeat_callback, 10)
        self.last_heartbeat_time = time.monotonic()
        self.timeout = 0.15  # 150 ms
        self.pin = 17
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        self.timer = self.create_timer(0.05, self.check_timeout)  # check every 50 ms

    def heartbeat_callback(self, msg):
        self.last_heartbeat_time = time.monotonic()

    def check_timeout(self):
        if time.monotonic() - self.last_heartbeat_time > self.timeout:
            GPIO.output(self.pin, GPIO.HIGH)
        else:
            GPIO.output(self.pin, GPIO.LOW)

def main(args=None):
    rclpy.init(args=args)
    node = EstopMonitor()
    rclpy.spin(node)
    node.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()