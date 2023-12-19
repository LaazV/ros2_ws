import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
import numpy as np
from scipy import ndimage
from std_msgs.msg import Header


class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')
        self.client = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        self.explore_goal = PoseStamped()
        self.map_data = None
        self.observation_threshold = 1.0
        self.get_logger().info('Nó de exploração inicializado.')

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info(f'Mapa recebido. Dimensões: {msg.info.width} x {msg.info.height}')
        self.send_explore_goal()

    def calculate_explore_goal(self):
        if self.map_data is not None:
            unexplored_cells = np.array(self.map_data.data) < 0

            if np.any(unexplored_cells):
                filtered_cells = self.filter_unexplored_cells(unexplored_cells)

                if np.any(filtered_cells):
                    centroid = ndimage.measurements.center_of_mass(filtered_cells)

                    x_centroid = self.map_data.info.origin.position.x + centroid[1] * self.map_data.info.resolution
                    y_centroid = self.map_data.info.origin.position.y + centroid[0] * self.map_data.info.resolution

                    self.update_goal_pose(x_centroid, y_centroid)
                else:
                    self.update_goal_pose(0.0, 2.0)
                    self.get_logger().info('Nenhuma célula não explorada encontrada. Enviando destino alternativo.')
            else:
                self.update_goal_pose(0.0, 2.0)
                self.get_logger().info('Nenhuma célula não explorada encontrada. Enviando destino alternativo.')
        else:
            self.update_goal_pose(0.0, 2.0)
            self.get_logger().info('Dados do mapa indisponíveis. Enviando destino padrão.')

    def filter_unexplored_cells(self, unexplored_cells):
        structuring_element = np.ones((3, 3))
        return ndimage.binary_opening(unexplored_cells, structure=structuring_element)

    def update_goal_pose(self, x, y):
        self.explore_goal.header = Header(frame_id='map')
        self.explore_goal.pose.position = Point(x=x, y=y, z=0.0)

    def send_explore_goal(self):
        self.calculate_explore_goal()
        self.client.publish(self.explore_goal)
        self.get_logger().info(f'Meta publicada em {self.client.topic_name}')


def main(args=None):
    rclpy.init(args=args)
    exploration_node = ExplorationNode()
    rclpy.spin(exploration_node)
    exploration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
