# ROS imports
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseArray, Pose, TransformStamped, PolygonStamped, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster

# Other imports
from .particle_filter import ParticleFilter
import math


def quaternion_from_euler(ai: float, aj: float, ak: float):
    temp = Quaternion()

    temp.x = math.sin(ai / 2) * math.cos(aj / 2) * math.cos(ak / 2) - math.cos(ai / 2) * math.sin(
        aj / 2) * math.sin(ak / 2)
    temp.y = math.cos(ai / 2) * math.sin(aj / 2) * math.cos(ak / 2) + math.sin(ai / 2) * math.cos(
        aj / 2) * math.sin(ak / 2)
    temp.z = math.cos(ai / 2) * math.cos(aj / 2) * math.sin(ak / 2) - math.sin(ai / 2) * math.sin(
        aj / 2) * math.cos(ak / 2)
    temp.w = math.cos(ai / 2) * math.cos(aj / 2) * math.cos(ak / 2) + math.sin(ai / 2) * math.sin(
        aj / 2) * math.sin(ak / 2)

    return temp


def euler_from_quaternion(q):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


class ParticleFilterNode(Node):
    MAX_PARTICLES = 500

    def __init__(self):
        super().__init__('Particle_Filter_Node')
        self.nodeName = self.get_name()
        # self.target_frame = self.declare_parameter('target_frame', 'turtle1')

        # self.map_filepath = self.declare_parameter('', '')
        # f = "/mnt/c/Users/ianso/Nextcloud/Source-Repo/Python/InHomeDelivery/src/localization/localization/maps/
        # new_map.json"
        # f = "/home/ian/src/InHomeDelivery/src/localization/localization/maps/new_map.json"
        f = "C:\\Users\\ianso\\Nextcloud\\Source-Repo\\Python\\InHomeDelivery\\src\\localization\\localization\\maps" \
            "\\new_map.json "
        self.map_filepath = f
        self.particle_filter = ParticleFilter(self.map_filepath)

        self.pose_array_subscription = self.create_subscription(
            PoseArray,
            'marker_poses',
            self.pose_updated,
            10)

        self.subscription = self.create_subscription(
            Odometry,
            'robot_odometry',
            self.odometry_updated,
            10)

        self.occupancy_grid_publisher = self.create_publisher(OccupancyGrid, 'occupancy_grid', 10)
        self.particles_publisher = self.create_publisher(PoseArray, 'particles', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'marker_array', 10)
        self.Robot_pose_publisher = self.create_publisher(PoseStamped, 'robot_pose', 10)

        # Call update function every half a second
        self.timer = self.create_timer(0.5, self.update)

    def update(self):
        self.publish_occupancy_grid()
        self.publish_marker_array()
        self.publish_particle_pose_array()
        # self.publish_robot_estimate()

    def run_particle_filter(self):
        # Obtain odometry information
        odom = self.particle_filter.compute_odometry(self.particle_filter.curr_pose)
        self.get_logger().info("odom:", odom)

        # Obtain list of currently seen markers and their poses
        # markers = await image_processing(robot)
        # marker_poses = cvt_2D_marker_measurements(markers)
        marker_poses = []
        self.get_logger().info("marker_poses:", marker_poses)

        # the key to the ros node version will be synchronizing the odometry collection and marker_pose timestamps

        # Update the particle filter using the obtained information
        x, y, h, confidence = self.particle_filter.update(odom, marker_poses)

        # Update the particle filter GUI for debugging
        self.get_logger().info('confidence:', confidence)
        # gui.show_mean(x, y, h)
        # gui.show_particles(pf.particles)
        # gui.updated.set()

        # Determine the robot’s actions based on the current state of the localization system
        if not confidence:
            # The search algorithm, the robot must be confident in its position to begin navigating
            # await robot.drive_straight(distance_mm(10), speed_mm/s(25), should_play_anim=False).wait_for_completed()
            # await robot.turn_in_place(degrees(25), speed=Angle(degrees=15.0)).wait_for_completed()
            pass
        else:
            pass

        # pf.last_pose = pf.curr_pose
        # pf.curr_pose = pf.robot.pose

    def publish_particle_pose_array(self):

        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'world'
        for i in range(min(len(self.particle_filter.particles), self.MAX_PARTICLES)):
            p = Pose()
            particle = self.particle_filter.particles[i]
            p.position.x = float(particle.x) * 0.01
            p.position.y = float(particle.y) * 0.01
            p.position.z = 0.0

            p.orientation = quaternion_from_euler(0.0, 0.0, particle.h)

            pa.poses.append(p)
        self.particles_publisher.publish(pa)
        self.get_logger().info('Publishing particle Poses')

    def publish_robot_estimate(self):
        p = PoseStamped()

        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'world'

        p.position.x = self.particle_filter.robot.x
        p.position.y = self.particle_filter.robot.y
        p.position.z = 0.0

        # Origin orientation
        p.orientation = quaternion_from_euler(0.0, 0.0, self.particle_filter.robot.h)

        self.Robot_pose_publisher.publish(p)

    def publish_occupancy_grid(self):
        og = OccupancyGrid()

        # configure header
        og.header.stamp = self.get_clock().now().to_msg()
        og.header.frame_id = 'world'

        # configure info
        og.info.map_load_time = self.get_clock().now().to_msg()
        og.info.resolution = self.particle_filter.grid.scale  # m/cell
        og.info.width = self.particle_filter.grid.width
        og.info.height = self.particle_filter.grid.height

        # Origin of the map
        map_origin = Pose()

        # Origin position
        map_origin.position.x = 0.0
        map_origin.position.y = 0.0
        map_origin.position.z = 0.0

        # Origin orientation
        map_origin.orientation = quaternion_from_euler(0.0, 0.0, 0.0)

        # assign origin to occupancy grid
        og.info.origin = map_origin

        # assign occupancy grid to data
        og.data = self.particle_filter.grid.occupancy_grid.flatten().astype(int).tolist()

        # publish occupancy grid
        self.occupancy_grid_publisher.publish(og)
        self.get_logger().info('Published occupancy_grid')

    def publish_marker_array(self):
        markers = MarkerArray()
        for m in self.particle_filter.grid.markers:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "main_ns"
            marker.id = 0
            marker.type = 1
            marker.action = 0
            marker.pose.position.x = float(m[0] * 0.01)
            marker.pose.position.y = float(m[1] * 0.01)
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            markers.markers.append(marker)
        self.marker_publisher.publish(markers)
        self.get_logger().info('Published marker_array')

    def pose_updated(self, data):
        pass

    def odometry_updated(self, data):
        pass


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ParticleFilterNode()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
