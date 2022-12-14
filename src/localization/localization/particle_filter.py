import random
import math
import json
import time

import numpy as np
import copy

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


class ParticleFilter:
    # odometry Gaussian noise model
    ODOM_TRANS_SIGMA = 0.01  # translational err in inch (grid unit)
    ODOM_HEAD_SIGMA = 0.5  # rotational err in deg

    # marker measurement Gaussian noise model
    MARKER_TRANS_SIGMA = 1.0  # translational err in inch (grid unit)
    MARKER_ROT_SIGMA = 20  # rotational err in deg

    ROBOT_CAMERA_FOV_DEG = 45  # Robot camera FOV in degree

    alpha1 = 0.001
    alpha2 = 0.001
    alpha3 = 0.005
    alpha4 = 0.005

    class CozGrid:

        min_occupancy = 50

        def __init__(self, filepath: str):
            self.filename = filepath

            self.map_time = time.perf_counter_ns()
            with open(self.filename) as config_file:
                config = json.loads(config_file.read())

            if config is None:
                raise FileNotFoundError("Could not open config file")

            self.config = config
            self.width = int(self.config['width'])
            self.height = int(self.config['height'])
            self.scale = float(self.config['scale'])

            obstacles = self.config['obstacles']
            self.obstacles = []
            for obstacle in obstacles:
                self.obstacles.append(Polygon(obstacle))

            self.markers = self.config['markers']

            self.occupancy_grid = np.zeros((self.height, self.width), np.int8)

            for i in range(len(self.occupancy_grid)):
                for j in range(len(self.occupancy_grid[0])):
                    for obstacle in self.obstacles:
                        if self.occupancy_grid[i][j] > 0:
                            break
                        if obstacle.contains(Point(i + 0.5, j + 0.5)):
                            self.occupancy_grid[i][j] = 100

        def is_in(self, x, y):
            """ Determine whether the cell is in the grid map or not
                Argument:
                x, y - X and Y in the cell map
                Return: boolean results
            """
            if x < 0 or y < 0 or x > self.width or y > self.height:
                return False
            return True

        def is_free(self, x, y):
            """ Determine whether the cell is in the *free part* of grid map or not
                Argument:
                x, y - X and Y in the cell map
                Return: boolean results
            """
            if not self.is_in(x, y):
                return False
            yy = int(y)  # self.height - int(y) - 1
            xx = int(x)
            return self.occupancy_grid[yy][xx] < self.min_occupancy

        def random_place(self):
            """ Return a random place in the map
                Argument: None
                Return: x, y - X and Y in the cell map
            """
            x = random.uniform(0, self.width)
            y = random.uniform(0, self.height)
            return x, y

        def random_free_place(self):
            """ Return a random place in the map which is free from obstacles
                Argument: None
                Return: x, y - X and Y in the cell map
            """
            while True:
                x, y = self.random_place()
                if self.is_free(x, y):
                    return x, y

    class Particle(object):

        # x = "X coordinate in world frame"
        # y = "Y coordinate in world frame"
        # h = "Heading angle in world frame in degree. h = 0 when robot's points to positive X"

        def __init__(self, x, y, heading=None):
            if heading is None:
                heading = random.uniform(0, 360)
            self.x = x
            self.y = y
            self.h = heading

        def __repr__(self):
            return "(x = %f, y = %f, heading = %f deg)" % (self.x, self.y, self.h)

        @property
        def xy(self):
            return self.x, self.y

        @property
        def xyh(self):
            return self.x, self.y, self.h

        @classmethod
        # create some random particles
        def create_random(cls, count, grid):
            return [cls(*grid.random_free_place()) for _ in range(0, count)]

        def move(self, rot1, trans, rot2):
            """
                Rotate the particle with rot1 degree and drive forward trans, and then rotate rot2 degree
                Note that the robot *turn first, then drive forward, followed by final turn*

                Arguments:
                rot1 -- degree to turn, turn left is positive
                trans -- distance to drive forward (unit in grid)
                rot2 -- degree to turn, turn left is positive

                No return
            """
            self.h = self.h + rot1
            dx = math.cos(math.radians(self.h)) * trans
            dy = math.sin(math.radians(self.h)) * trans
            self.x += dx
            self.y += dy
            self.h = self.h + rot2

        def read_markers(self, grid):
            """ Helper function to simulate markers measurements by robot's camera
                Only markers in robot's camera view (in FOV) will be in the list

                Arguments:
                grid -- map grid with marker information

                Return: robot detected marker list, each marker has format:
                        measured_marker_list[i] = (rx, ry, rh)
                        rx -- marker's relative X coordinate in robot's frame
                        ry -- marker's relative Y coordinate in robot's frame
                        rh -- marker's relative heading in robot's frame, in degree
            """
            marker_list = []
            for marker in grid.markers:
                # m_x, m_y, m_h = self.parse_marker_info(marker[0], marker[1], marker[2])
                m_x = marker[0]
                m_y = marker[1]
                m_h = marker[2]
                # rotate marker into robot frame
                mr_x, mr_y = ParticleFilter.rotate_point(m_x - self.x, m_y - self.y, -self.h)
                if math.fabs(math.degrees(math.atan2(mr_y, mr_x))) < ParticleFilter.ROBOT_CAMERA_FOV_DEG / 2.0:
                    mr_h = m_h - self.h
                    marker_list.append((mr_x, mr_y, mr_h))
            return marker_list

        @staticmethod
        def parse_marker_info(col, row, heading_char):
            c = None
            r = None
            heading = None
            if heading_char == 'U':
                c = col + 0.5
                r = row
                heading = 90
            elif heading_char == 'D':
                c = col + 0.5
                r = row + 1
                heading = 270
            elif heading_char == 'L':
                c = col + 1
                r = row + 0.5
                heading = 180
            elif heading_char == 'R':
                c = col
                r = row + 0.5
                heading = 0
            if not c or not r or not heading:
                raise ValueError("incorrect marker value")
            return c, r, heading

    class Robot(Particle):

        def __init__(self, x, y, h):
            super(ParticleFilter.Robot, self).__init__(x, y, h)

        def __repr__(self):
            return "(x = %f, y = %f, heading = %f deg)" % (self.x, self.y, self.h)

        @property
        def pose(self):
            return [self.x, self.y, self.h]

        # return a random robot heading angle
        @staticmethod
        def chose_random_heading():
            return random.uniform(0, 360)

        def check_collision(self, rot1, trans, rot2, grid):
            """ Check whether moving the robot will cause collision.
                Note this function will *not* move the robot

                Arguments:
                rot1 -- degree to turn, turn left is positive
                trans -- distance to drive forward (unit in grid)
                rot2 -- degree to turn, turn left is positive

                Return: True if movement will cause collision, False if movement will not be a collision
            """
            h = self.h + rot1
            dx = math.cos(math.radians(h)) * trans
            dy = math.sin(math.radians(h)) * trans
            if grid.is_free(self.x + dx, self.y + dy):
                return False
            return True

    def __init__(self, map_filename, particle_count=5000):
        self.PARTICLE_COUNT = particle_count
        self.grid = self.CozGrid(map_filename)
        self.particles = self.Particle.create_random(self.PARTICLE_COUNT, self.grid)
        # TODO: check this robot thing here
        x, y = self.grid.random_free_place()
        self.robot = self.Robot(x, y, random.randint(0, 360))
        self.last_pose = None
        self.curr_pose = None

    def update(self, odom, r_marker_list):
        # ---------- Motion model update ----------
        self.motion_update(odom)

        # ---------- Sensor (markers) model update ----------
        self.measurement_update(r_marker_list)

        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = self.compute_mean_pose()
        return m_x, m_y, m_h, m_confident

    def motion_update(self, odom):
        """ Particle filter motion update

            Arguments:
            particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                    before motion update
            odom -- noisy odometry measurements, a pair of robot pose, i.e. last time
                    step pose and current time step pose

            Returns: the list of particle representing belief tilde{p}(x_{t} | u_{t})
                    after motion update
        """

        x_t = odom[1]  # represents pose at time t
        x_t_min1 = odom[0]  # represents pose at time t-1

        # Calculate initial rotation of robot
        s_rot1 = math.degrees(math.atan2(x_t[1] - x_t_min1[1], x_t[0] - x_t_min1[0])) - x_t_min1[2]
        # Calculate translation of robot
        s_trans = math.sqrt(math.pow(x_t_min1[0] - x_t[0], 2) + math.pow(x_t_min1[1] - x_t[1], 2))
        # Calculate the final rotation of the robot
        s_rot2 = x_t[2] - x_t_min1[2] - s_rot1

        # for each particle
        for particle in self.particles:
            # Calculate initial rotation for particle
            # TODO: determine if i need to calculate motion model like this since i am using an omni bot
            sb_rot1 = s_rot1 - self.sample((self.alpha1 * s_rot1) + (self.alpha2 * s_trans))
            # Calculate translation for particle
            sb_trans = s_trans - self.sample((self.alpha3 * s_trans) + (self.alpha4 * (s_rot1 + s_rot2)))
            # Calculate final rotation for particle
            sb_rot2 = s_rot2 - self.sample((self.alpha1 * s_rot2) + (self.alpha2 * s_trans))

            # Move the particle
            particle.move(sb_rot1, sb_trans, sb_rot2)

    def measurement_update(self, measured_marker_list):
        """ Particle filter measurement update

        Arguments:
        particles -- a list of particle represents belief tilde{p}(x_{t} | u_{t})
                before measurement update
        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree
        grid -- grid world map containing the marker information.
                see grid.py and CozGrid for definition

        Returns: the list of particle representing belief p(x_{t} | u_{t})
                after measurement update
        """
        if len(measured_marker_list) == 0:
            return
        measured_particles = []
        weights = []
        # find score for each particle
        for particle in self.particles:
            # if the particle is not in grid, skip
            p_marks = particle.read_markers(self.grid)
            q_values = []
            # need condition if len(p_marks == 0, but len of robot markers != 0)
            if len(p_marks) == 0:
                q_values = [0]
            # if the particle is not in the grid, ignore
            elif not self.grid.is_in(particle.x, particle.y):
                q_values = [0]
                new_pos = self.grid.random_place()
                particle.x = new_pos[0]
                particle.y = new_pos[1]
                particle.h = random.randint(0, 360)
            else:
                for mark in p_marks:
                    mark_x = mark[0]
                    mark_y = mark[1]
                    # calculate radius
                    r = math.sqrt(math.pow(mark_x, 2) +
                                  math.pow(mark_y, 2))
                    # calculate bearing
                    theta = math.atan2(mark_y, mark_x)
                    # signature?

                    # for every marker the robot see's, compare this marker's radius and theta against it
                    q_marker_values = [1]
                    # need condition if len(robot_markers) == 0
                    for robot_mark in measured_marker_list:
                        robot_r = math.sqrt(math.pow(robot_mark[0], 2) + math.pow(robot_mark[1], 2))
                        robot_theta = math.atan2(robot_mark[1], robot_mark[0])
                        delta_r = r - robot_r
                        delta_theta = theta - robot_theta
                        delta_h = (robot_mark[2] % 360) - (mark[2] % 360)

                        q = self.norm_pdf(delta_r, 0, self.MARKER_TRANS_SIGMA) * \
                            self.norm_pdf(math.degrees(delta_theta), 0, self.MARKER_ROT_SIGMA) * \
                            self.norm_pdf(delta_h, 0, self.MARKER_ROT_SIGMA)
                        q_marker_values.append(q)

                    # find the highest probability (most likely marker seen by particle and robot are the same) (not
                    # sure if this is the right way to do this, versus just multiplying all the deltas) However,
                    # multiplying all the deltas would return a low prob if one marker is very close to another (
                    # correctly so) but Extremely far from a separate marker (also correctly so, since they are not
                    # the same markers)
                    q_values.append(np.prod(q_marker_values))

            # product of each prob on prb list
            particle_probability = np.prod(q_values)
            measured_particles.append(particle)
            weights.append(particle_probability)

        # normalize weights
        weights = np.array(weights)
        weights[np.isnan(weights)] = 0
        sum_t = np.sum(weights)
        if sum_t:
            norm_weights = weights / np.sum(weights)
        else:
            # return particles
            norm_weights = weights
            return self.Particle.create_random(self.PARTICLE_COUNT, self.grid)

        return_particles = []
        indices = [np.random.choice(
            np.arange(0, len(measured_particles)), p=norm_weights) for _ in range(len(measured_particles))]

        for i in indices:
            return_particles.append(copy.copy(measured_particles[i]))

        # resample new particle (give weights and resample according to weights)
        # measured_particles = list of tuples of particles w/ corresponding probability value
        return return_particles

    def compute_mean_pose(self, confident_dist=1):
        """
        Compute the mean for all particles that have a reasonably good weight.
        This is not part of the particle filter algorithm but rather an
        addition to show the "best belief" for current position.
        """
        m_x, m_y, m_count = 0, 0, 0
        # for rotation average
        m_hx, m_hy = 0, 0
        for p in self.particles:
            m_count += 1
            m_x += p.x
            m_y += p.y
            m_hx += math.sin(math.radians(p.h))
            m_hy += math.cos(math.radians(p.h))

        if m_count == 0:
            return -1, -1, 0, False

        m_x /= m_count
        m_y /= m_count

        # average rotation
        m_hx /= m_count
        m_hy /= m_count
        m_h = math.degrees(math.atan2(m_hx, m_hy))

        # Now compute how good that mean is -- check how many particles
        # actually are in the immediate vicinity
        m_count = 0
        for p in self.particles:
            if self.grid_distance(p.x, p.y, m_x, m_y) < confident_dist:
                m_count += 1

        return m_x, m_y, m_h, m_count > len(self.particles) * 0.95

    def compute_odometry(self, cvt_inch=True):
        last_x, last_y, last_h = self.last_pose.position.x, self.last_pose.position.y, \
                                 self.last_pose.rotation.angle_z.degrees
        curr_x, curr_y, curr_h = self.curr_pose.position.x, self.curr_pose.position.y, self.curr_pose.rotation.angle_z.degrees

        if cvt_inch:
            last_x, last_y = last_x / 25.6, last_y / 25.6
            curr_x, curr_y = curr_x / 25.6, curr_y / 25.6

        return [[last_x, last_y, last_h], [curr_x, curr_y, curr_h]]

    @staticmethod
    def sample(x):
        return random.gauss(0, x)

    @staticmethod
    def norm_pdf(x, mean, sd):
        var = float(sd) ** 2
        denom = (2 * math.pi * var) ** .5
        num = math.exp(-(float(x) - float(mean)) ** 2 / (2 * var))
        return num / denom

    @staticmethod
    def grid_distance(x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    @staticmethod
    def rotate_point(x, y, heading_deg):
        c = math.cos(math.radians(heading_deg))
        s = math.sin(math.radians(heading_deg))
        xr = x * c + y * -s
        yr = x * s + y * c
        return xr, yr


def main(particle_filter, robot):
    particle_filter.last_pose = robot.pose
    particle_filter.curr_pose = particle_filter.last_pose
    while True:
        # Obtain odometry information
        odom = particle_filter.compute_odometry(particle_filter.curr_pose)
        print("INFO: odom", odom)

        # Obtain list of currently seen markers and their poses
        # markers = await image_processing(robot)
        # marker_poses = cvt_2D_marker_measurements(markers)
        marker_poses = []
        print("INFO: marker_poses", marker_poses)

        # the key to the ros node version will be synchronizing the odometry collection and marker_pose timestamps

        # Update the particle filter using the obtained information
        x, y, h, confidence = particle_filter.update(odom, marker_poses)

        # Update the particle filter GUI for debugging
        print('INFO: confidence', confidence)
        print("")
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

        pf.last_pose = pf.curr_pose
        pf.curr_pose = pf.robot.pose


if __name__ == "__main__":
    # f = "/mnt/c/Users/ianso/Nextcloud/Source-Repo/Python/InHomeDelivery/src/localization/localization/map_arena.json"
    f = "map_arena.json"
    pf = ParticleFilter(f)
    # main(pf)
