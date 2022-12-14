
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('Camera_subscriber')

        # ROS Subscribers
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.listener_callback,
            10)

        # OpenCV
        self.br = CvBridge()

    def listener_callback(self, data):
        self.get_logger().info('Received image')

        current_frame = self.br.imgmsg_to_cv2(data)

        cv2.imshow("camera", current_frame)

        cv2.waitKey(1)


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
