import rclpy

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2
import numpy as np
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from YOLODetector import YOLODetector

class ImageReaderNode:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('image_reader_node')
        
        #Create subscribers
        self.pose_subscriber = self.create_subscription(Odometry, "odom", self.get_pose, 10)
        self.subscription = self.node.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.laser_subscriber = self.create_subscription(LaserScan, "scan", self.get_ranges, 10)

        #Create publisher
        self.publisher_ = self.create_publisher(Bool, 'stop', 10)
        self.sync_timer = 5
        self.timer_publisher = self.create_timer(self.sync_timer, self.pub_callback)

        #Posições
        self.x = None
        self.y = None
        self.yaw = None

        self.yolo = YOLODetector()

        self.fov_camera = 30
        self.stop = False

    def print_bbox(self, image, results):
        # Desenhar as bounding boxes na imagem
        for result in results:
            x, y, w, h = result['bbox']
            # label = result['label']
            confidence = result['confidence']

            # Desenhar a bounding box retangular
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            #desenhe um centroid no centro da bounding box
            cv2.circle(image, (int(x + w/2), int(y + h/2)), 5, (0, 255, 0), -1)

            # Escrever o rótulo e a confiança da bounding box
            # text = f"{label}: {confidence:.2f}"
            cv2.putText(image, str(confidence), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # Converter RGB para BGR
        # image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Exibir a imagem com as bounding boxes
        cv2.imshow("Image with Bounding Boxes", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def get_ranges(self, msg):
        self.ranges = msg.ranges

    def get_pose(self, msg):
        pose = msg.pose.pose

        self.x =  pose.position.x
        self.y =  pose.position.y
        _, _, self.yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    def pub_callback(self):
        msg = Bool()
        msg.data = self.stop
        self.publisher_.publish(msg)

    def image_callback(self, msg):
        # Convert the ROS Image message to a numpy array    
        height = msg.height
        width = msg.width
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, -1)
        bgr_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # bgr_image = cv2.imread('./yolo/test_images/hospital_world.jpg')
        results = self.yolo.detector(bgr_image)
        if results:
            best_result = max(results, key=lambda x: x['confidence'])
            centroid = (int(best_result['bbox'][0] + best_result['bbox'][2]/2), int(best_result['bbox'][1] + best_result['bbox'][3]/2))
            print('Pessoa Encontrada!')
            print(f'Posição do robô: ({self.x}, {self.y})')

            # Calcular a distância entre o robô e a pessoa
            delta_x_pixels = centroid[0] - width/2
            delta_x_degrees = self.fov_camera * delta_x_pixels / width #Entre -15 e 15 graus

            if delta_x_degrees > 0:
                index_laser = int(delta_x_degrees)
            else:
                index_laser = int(360 - delta_x_degrees)

            distance = self.ranges[index_laser]
            
            ang_rad = (index_laser * np.pi / 180) + self.yaw
            x = distance * np.cos(ang_rad) + self.x
            y = distance * np.sin(ang_rad) + self.y
            person_coord = (x, y)
            print(f'Posição da pessoa: {person_coord}')

            self.stop = True

            self.print_bbox(bgr_image, [best_result])

    def run(self):
        rclpy.spin(self.node)

if __name__ == '__main__':
    node = ImageReaderNode()
    node.run()
