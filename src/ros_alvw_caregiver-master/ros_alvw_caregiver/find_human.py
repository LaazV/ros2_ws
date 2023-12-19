import rclpy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time

class FindHumanNode:

    def __init__(self):
        self.node = rclpy.create_node('find_human_node')
        self.node.get_logger().info('Find Human iniciado!')
        # Tópico para parar a exploração.
        self.publisher = self.node.create_publisher(Bool, '/explore/resume', 10)

        # Tópico para enviar a posição desejada.
        self.goal_publisher = self.node.create_publisher(PoseStamped, '/goal_pose', 10)

        # Assina o tópico de odometria para obter a posição atual do robô.
        self.odom_subscription = self.node.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        self.achei_subscription = self.node.create_subscription(
            Bool, '/achei', self.achei_callback, 10)


    def achei_callback(self, msg):
        if msg.data is True:
            self.node.get_logger().info('Encontrei o paciente!')
            self.node.get_logger().info('Posição do robô publicada. Encerrando o nó...')
            time.sleep(1)
            msg_stop = Bool()
            msg_stop.data = False
            self.publisher.publish(msg_stop)

            time.sleep(1)

            # Comando para mover o robô para a posição desejada (x=2, y=1).
            self.move_robot_to_position()

            time.sleep(1)
        # self.node.get_logger().info("NÃO ENCONTREI PACIENTE -----")


    def odom_callback(self, msg):
        # Função de retorno de chamada para obter a posição atual do robô.
        current_position = msg.pose.pose.position
        # self.node.get_logger().info(f'Posição atual do robô: x={current_position.x}, y={current_position.y}')

        # Após receber a informação da odometria, encerra o nó.

    def move_robot_to_position(self):
        # Comando para mover o robô para a posição desejada.
        position_msg = PoseStamped()
        position_msg.header.stamp = self.node.get_clock().now().to_msg()
        position_msg.header.frame_id = 'map'
        position_msg.pose.position.x = 0.0
        position_msg.pose.position.y = 2.0
        position_msg.pose.position.z = 0.0
        position_msg.pose.orientation.x = 0.0
        position_msg.pose.orientation.y = 0.0
        position_msg.pose.orientation.z = 0.0
        position_msg.pose.orientation.w = 1.0

        self.goal_publisher.publish(position_msg)

def main():
    rclpy.init()
    find_human_node = FindHumanNode()
    rclpy.spin(find_human_node.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
