import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Pose
from enum import Enum
import time
import os


class State(Enum):
    SEARCHING = 0
    APPROACHING = 1
    FOLLOWING = 2
    SIDE_STOP = 3


class PersonFollowFSM(Node):

    def __init__(self):
        super().__init__('person_follow_fsm')

        # Parâmetro: lado de parada
        self.declare_parameter('side', 'right')
        self.side = self.get_parameter('side').get_parameter_value().string_value

        # Estados
        self.state = State.SEARCHING
        self.last_state = None
        self.stop_counter = 0

        # Flags
        self.person_detected = False
        self.person_found_sent = False
        self.mission_done_sent = False

        # Tempo
        self.start_time = time.time()

        # Pose real da pessoa (vinda do detector)
        self.latest_person_pose = None
        self.person_x = None
        self.person_y = None

        # Subscribers
        self.detect_sub = self.create_subscription(
            Bool,
            '/person_detected',
            self.person_detected_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            Pose,
            '/person_pose',
            self.person_pose_callback,
            10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.done_pub = self.create_publisher(Bool, '/mission_done', 10)
        self.found_pub = self.create_publisher(Bool, '/person_found', 10)

        # Timer
        self.timer = self.create_timer(1.0, self.update)

        self.get_logger().info(f'FSM iniciada. Lado escolhido: {self.side}')

    def person_detected_callback(self, msg: Bool):
        self.person_detected = msg.data

    def person_pose_callback(self, msg: Pose):
        self.latest_person_pose = msg

    def publish_found_once(self):
        if not self.person_found_sent:
            # Guarda a localização aproximada da pessoa no momento em que foi "encontrada"
            if self.latest_person_pose is not None:
                self.person_x = float(self.latest_person_pose.position.x)
                self.person_y = float(self.latest_person_pose.position.y)
            else:
                # Se não chegou pose ainda, grava como desconhecido
                self.person_x = None
                self.person_y = None

            msg = Bool()
            msg.data = True
            self.found_pub.publish(msg)

            self.person_found_sent = True
            if self.person_x is None:
                self.get_logger().info('✅ Pessoa localizada! (pose ainda não disponível)')
            else:
                self.get_logger().info(f'✅ Pessoa localizada! (x≈{self.person_x:.2f}, y≈{self.person_y:.2f})')

    def publish_done_once(self):
        if not self.mission_done_sent:
            msg = Bool()
            msg.data = True
            self.done_pub.publish(msg)
            self.mission_done_sent = True
            self.generate_report()

    def generate_report(self):
        total_time = time.time() - self.start_time
        report_path = os.path.expanduser('~/mission_report.txt')

        with open(report_path, 'w') as f:
            f.write('Mission Report\n')
            f.write('==============\n')
            f.write('Person found at (approx):\n')

            if self.person_x is None or self.person_y is None:
                f.write('  x = unknown\n')
                f.write('  y = unknown\n\n')
            else:
                f.write(f'  x = {self.person_x:.2f} m\n')
                f.write(f'  y = {self.person_y:.2f} m\n\n')

            f.write(f'Total mission time: {total_time:.2f} seconds\n')

        self.get_logger().info(f'📄 Relatório salvo em {report_path}')

    def update(self):
        twist = Twist()

        if self.state == State.SEARCHING:
            self.stop_counter = 0
            if self.person_detected:
                self.publish_found_once()
                self.state = State.APPROACHING

        elif self.state == State.APPROACHING:
            twist.linear.x = 0.3
            if self.person_detected:
                self.state = State.FOLLOWING

        elif self.state == State.FOLLOWING:
            twist.linear.x = 0.2
            self.state = State.SIDE_STOP

        elif self.state == State.SIDE_STOP:
            if self.side == 'left':
                twist.angular.z = 0.4
            else:
                twist.angular.z = -0.4

            twist.linear.x = 0.1
            self.stop_counter += 1

            if self.stop_counter > 3:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info('Parada lateral concluída')
                self.state = None

        if self.state != self.last_state:
            if self.state is None:
                self.get_logger().info('Estado: DONE')
                self.publish_done_once()
            else:
                self.get_logger().info(f'Estado: {self.state.name}')
            self.last_state = self.state

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = PersonFollowFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
