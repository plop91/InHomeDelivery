"""
Camera subscriber Node
Ian Sodersjerna
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class ImageSubscriber(Node):
    """
    Image subscriber node
    """

    def __init__(self):
        """
        Initialize subscriber node
        """
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
        """
        Listener callback, displays provided image.
        :param data:
        :return:
        """

        # Log that we got a message
        self.get_logger().info('Received image')

        # convert from ROS format to cv2 format
        current_frame = self.br.imgmsg_to_cv2(data)

        # display image
        cv2.imshow("camera", current_frame)
        # cv2.waitKey(1)


def main(args=None):
    """
    Main function of the program, initializes ROS.
    :param args: arguments passed to the program by ROS
    :return: exit code
    """
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

    return 0


if __name__ == '__main__':
    exit(main())
