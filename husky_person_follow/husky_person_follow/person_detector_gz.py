import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool


class PersonDetectorGZ(Node):
    """
    Detector baseado em Gazebo (ground truth):
    - assina a pose bridgeada do Gazebo (tópico configurável)
    - publica /person_pose (Pose)
    - publica /person_detected (Bool) como True quando recebe pose
    """

    def __init__(self):
        super().__init__('person_detector_gz')

        # Tópico de entrada (pose bridgeada do Gazebo)
        self.declare_parameter('gz_pose_topic', '/world/office/model/person/pose')
        self.gz_pose_topic = self.get_parameter('gz_pose_topic').get_parameter_value().string_value

        self.pose_pub = self.create_publisher(Pose, '/person_pose', 10)
        self.detect_pub = self.create_publisher(Bool, '/person_detected', 10)

        self.sub = self.create_subscription(
            Pose,
            self.gz_pose_topic,
            self.pose_callback,
            10
        )

        self.get_logger().info(f'GZ detector iniciado. Lendo pose de: {self.gz_pose_topic}')

    def pose_callback(self, msg: Pose):
        # Republisha como pose “da pessoa” para o restante do sistema
        self.pose_pub.publish(msg)

        detected = Bool()
        detected.data = True
        self.detect_pub.publish(detected)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectorGZ()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
