"""
Camera Node
Ian Sodersjerna
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, Pose
from std_srvs.srv import Empty

import cv2
import numpy as np
import math
import time

aruco_dict = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}


def quaternion_from_euler(ai: float, aj: float, ak: float):
    """
    Convert euler coordinate to quaternion.
    :param ai: i component of euler number
    :param aj: j component of euler number
    :param ak: k component of euler number
    :return:
    """
    roll = ai
    pitch = aj
    yaw = ak

    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(
        pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(
        pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(
        pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(
        pitch / 2) * math.sin(yaw / 2)

    return [qx, qy, qz, qw]


class CameraNode(Node):
    """
    The camera node is responsible for publishing camera images as well as marker transforms.
    """

    def __init__(self, calibration_matrix_path, distortion_matrix_path, dict_type: str = "DICT_ARUCO_ORIGINAL"):
        """
        Initialize a camera node
        :param calibration_matrix_path: path to calibration matrix
        :param distortion_matrix_path: path to distortion matrix
        :param dict_type: aruco dict type as string
        """
        super().__init__('Camera_Node')

        # ROS parameter
        self.declare_parameter('frame_id', 'world')

        # Variables
        self.timer_period = 0.1  # seconds
        self.marker_poses = []
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publishing_raw_image = True
        self.publishing_marker_image = False
        self.estimating_pose = False

        # ROS Publishers
        self.img_publisher = self.create_publisher(Image, 'video_frames', 10)
        self.pose_publisher = self.create_publisher(PoseArray, 'marker_poses', 10)
        self.pose_img_publisher = self.create_publisher(Image, 'marker_pose_frames', 10)

        # ROS Services
        self.enable_raw_img_srv = self.create_service(Empty, 'enable_raw_image_publishing', self.enable_raw_img_pub)
        self.disable_raw_img_srv = self.create_service(Empty, 'disable_raw_image_publishing', self.disable_raw_img_pub)

        self.enable_marker_img_srv = self.create_service(Empty, 'enable_marker_image_publishing',
                                                         self.enable_marker_img_pub)
        self.disable_marker_img_srv = self.create_service(Empty, 'disable_marker_image_publishing',
                                                          self.disable_marker_img_pub)

        self.enable_pose_estimation_srv = self.create_service(Empty, 'enable_pose_publishing',
                                                              self.enable_pose_estimation)
        self.disable_pose_estimation_srv = self.create_service(Empty, 'disable_pose_publishing',
                                                               self.disable_pose_estimation)

        # OpenCV
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.br = CvBridge()

        self.aruco_dict_type = aruco_dict[dict_type]
        self.aruco_dict = cv2.aruco.Dictionary_get(self.aruco_dict_type)
        self.parameters = cv2.aruco.DetectorParameters_create()

        try:
            self.calibration_matrix_path = calibration_matrix_path
            self.calibration_matrix = np.load(self.calibration_matrix_path)
        except FileNotFoundError:
            raise Exception('calibration matrix not found')

        try:
            self.distortion_coefficients_path = distortion_matrix_path
            self.distortion_coefficients = np.load(self.distortion_coefficients_path)

        except FileNotFoundError:
            raise Exception('distortion coefficient not found')

        # ROS Timer
        self.timer = self.create_timer(self.timer_period, self.update)

    def update(self):
        """
        Update function, reads an image, then publishes it, then estimates poses of markers in image
        :return: None
        """
        # start timing
        t1 = time.perf_counter()

        # get a frame from the camera
        ret, frame = self.cap.read()

        # if a frame is present
        if ret:
            if self.publishing_raw_image:
                self.publish_raw_image(frame)
            # if we are estimating poses
            if self.estimating_pose:

                # estimate poses
                updated_frame = self.pose_estimation(frame)

                # publish poses
                self.publish_poses()
                if self.publishing_marker_image:
                    self.publish_marker_image(updated_frame)
        total = time.perf_counter() - t1
        self.get_logger().info(f'Update took: {total} sec')

    def publish_raw_image(self, frame):
        """
        Publish a raw frame with no image processing
        :param frame: Frame to be published
        :return: None
        """
        self.get_logger().info('Publishing raw frame')
        self.img_publisher.publish(self.br.cv2_to_imgmsg(frame))

    def publish_marker_image(self, frame):
        """
        Publish a frame with markers highlighted
        :param frame: frame to be published
        :return: None
        """
        self.get_logger().info('Publishing marker frame')
        self.pose_img_publisher.publish(self.br.cv2_to_imgmsg(frame))

    def pose_estimation(self, frame):
        """
        frame - Frame from the video stream
        matrix_coefficients - Intrinsic matrix of the calibrated camera
        distortion_coefficients - Distortion coefficients associated with your camera

        return:-
        frame - The frame with the axis drawn on it
        """
        self.get_logger().info('Estimating Poses')

        # convert to greyscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # detect markers in the image
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters,
                                                                    cameraMatrix=self.calibration_matrix,
                                                                    distCoeff=self.distortion_coefficients)

        # create marker pose list
        marker_poses = []

        # If markers are detected
        if len(corners) > 0:
            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec
                rvec, tvec, marker_points = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02,
                                                                                self.calibration_matrix,
                                                                                self.distortion_coefficients)
                marker_poses.append((rvec, tvec, marker_points, ids[i]))

                # Draw a square around the markers
                cv2.old.drawDetectedMarkers(frame, corners)

                # Draw Axis
                cv2.aruco.drawAxis(frame, self.calibration_matrix, self.distortion_coefficients, rvec, tvec, 0.01)

        # print("marker poses unsorted:", marker_poses)
        marker_poses.sort(key=lambda x: x[3])
        # print("marker poses sorted:", marker_poses)
        self.marker_poses = marker_poses

        return frame

    def publish_poses(self):
        """
        Publishes poses of markers in image.
        :return: None
        """
        self.get_logger().info('Publishing marker poses')

        pa = PoseArray()

        # Read message content and assign it to
        # corresponding tf variables
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = self.frame_id

        for i in range(len(self.marker_poses)):
            p = Pose()
            # Turtle only exists in 2D, thus we get x and y translation
            # coordinates from the message and set the z coordinate to 0
            # TODO: figure out this nested list nonsense
            p.position.x = float(self.marker_poses[i][1][0][0][0])
            p.position.y = float(self.marker_poses[i][1][0][0][1])
            p.position.z = float(self.marker_poses[i][1][0][0][2])

            # For the same reason, turtle can only rotate around one axis
            # and this why we set rotation in x and y to 0 and obtain
            # rotation in z axis from the message
            q = quaternion_from_euler(self.marker_poses[i][0][0][0][0], self.marker_poses[i][0][0][0][1],
                                      self.marker_poses[i][0][0][0][2])
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            pa.poses.append(p)

            if i == 1:
                self.get_logger().info('x: %s' % p.position.x)

        self.pose_publisher.publish(pa)

    def enable_raw_img_pub(self, request, response):
        """
        Service callback to enable raw image publishing.
        :param request: request message
        :param response: response message
        :return: response to the service call.
        """
        self.publishing_raw_image = True
        return response

    def disable_raw_img_pub(self, request, response):
        """
        Service callback to disable raw image publishing.
        :param request: request message
        :param response: response message
        :return: response to the service call.
        """
        self.publishing_raw_image = False
        return response

    def enable_marker_img_pub(self, request, response):
        """
        Service callback to enable marker image publishing.
        :param request: request message
        :param response: response message
        :return: response to the service call.
        """
        self.publishing_marker_image = True
        return response

    def disable_marker_img_pub(self, request, response):
        """
        Service callback to disable marker image publishing.
        :param request: request message
        :param response: response message
        :return: response to the service call.
        """
        self.publishing_marker_image = False
        return response

    def enable_pose_estimation(self, request, response):
        """
        Service callback to enable marker pose publishing.
        :param request: request message
        :param response: response message
        :return: response to the service call.
        """
        self.estimating_pose = True
        return response

    def disable_pose_estimation(self, request, response):
        """
        Service callback to disable marker pose publishing.
        :param request: request message
        :param response: response message
        :return: response to the service call.
        """
        self.estimating_pose = False
        return response


def main(args=None):
    """
    Main function of the program, sets path for the calibration matrix and dist coefficients and calls to initialize ROS
    :param args: arguments passed to the program by ROS
    :return: exit code
    """

    # Initialize the rclpy library
    rclpy.init(args=args)

    cal_matrix = "/home/ian/src/InHomeDelivery/src/camera/camera" \
                 "/data/calibration_matrix.npy"
    dist_coef = "/home/ian/src/InHomeDelivery/src/camera/camera" \
                "/data/distortion_coefficients.npy"

    # Create the node
    image_publisher = CameraNode(cal_matrix, dist_coef)

    # Spin the node so the callback function is called.
    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

    return 0


if __name__ == '__main__':
    exit(main())
