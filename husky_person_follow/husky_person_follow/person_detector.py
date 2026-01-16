import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class PersonDetector(Node):

    def __init__(self):
        super().__init__('person_detector')

        self.publisher_ = self.create_publisher(Bool, '/person_detected', 10)
        self.done_sub = self.create_subscription(Bool, '/mission_done', self.done_callback, 10)

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.detected = False
        self.mission_done = False

        self.get_logger().info('Person detector started')

    def done_callback(self, msg: Bool):
        if msg.data:
            self.mission_done = True
            self.get_logger().info('Mission done received. Shutting down detector...')
            rclpy.shutdown()

    def timer_callback(self):
        if self.mission_done:
            return

        self.detected = not self.detected
        msg = Bool()
        msg.data = self.detected
        self.publisher_.publish(msg)
        self.get_logger().info(f'Person detected: {self.detected}')


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
