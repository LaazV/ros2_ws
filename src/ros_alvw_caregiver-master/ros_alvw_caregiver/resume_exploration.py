import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ResumeExplorationNode(Node):
    def __init__(self):
        super().__init__('resume_exploration_node')
        self.init_publisher()
        self.timer = self.create_timer(5.0, self.publish_cmd_vel)  # Cria um timer de 5 segundos

    def init_publisher(self):
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def publish_cmd_vel(self): # Publica a velocidade apenas uma vez, após 5 segs
        twist_msg = Twist()
        twist_msg.angular.z = 0.3
        twist_msg.linear.x = 0.2

        self.velocity_publisher.publish(twist_msg)
        self.get_logger().info("Published cmd_vel. Shutting down...")

        # Desabilita o nó após a primeira publicação
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    resume_exploration_node = ResumeExplorationNode()
    rclpy.spin(resume_exploration_node)


if __name__ == '__main__':
    main()
