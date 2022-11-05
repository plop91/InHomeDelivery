#!/usr/bin/env python3
import math
import time
from grid import *
from particle import Particle
from utils import *
from setting import *
from scipy.stats import norm
import numpy as np
import copy


# particle filter functionality
class ParticleFilter:

    def __init__(self, map_filename, goal=(6, 10, 0)):

        self.map_filename = map_filename
        self.grid = CozGrid(self.map_filename)
        # TODO: check if map loaded

        self.particles = Particle.create_random(PARTICLE_COUNT, self.grid)

        # TODO: setup a better default goal
        self.goal = goal

        # TODO: setup better last pose and curr pose defaults
        self.curr_pose = None
        self.last_pose = self.curr_pose

    async def run(self):
        # TODO: check if receiving images, if not wait and log.

        ############################################################################
        # TODO: get robot pose
        # self.last_pose = robot.pose
        self.curr_pose = self.last_pose
        while True:
            # Make your code robust to the “kidnapped robot problem” by resetting localization

            # Obtain odometry information
            # TODO: retrieve odometry info
            # odom = compute_odometry(curr_pose)
            odom = None
            print("INFO: odom", odom)

            # Obtain list of currently seen markers and their poses
            # TODO: retrieve marker poses
            # markers = await image_processing(robot)
            # marker_poses = cvt_2d_marker_measurements(markers)
            marker_poses = []
            print("INFO: marker_poses", marker_poses)

            # Update the particle filter
            x, y, h, confidence = self.update(odom, marker_poses)

            # Update the particle filter GUI for debugging
            print('INFO: confidence', confidence)
            print("")
            # TODO: GUI stuff
            # gui.show_mean(x, y, h)
            # gui.show_particles(pf.particles)
            # gui.updated.set()

            # Determine the robot’s actions based on the current state of the localization system
            if not confidence:
                # await robot.drive_straight(distance_mm(10), speed_mmps(25), should_play_anim=False)
                # .wait_for_completed()
                # await robot.turn_in_place(degrees(25), speed=Angle(degrees=15.0)).wait_for_completed()
                # TODO: locate ar tag protocol
                pass
            else:
                # Have the robot drive to the goal
                distance = grid_distance(x, y, self.goal[0], self.goal[1])
                # print("distance", distance)

                x_diff = x - self.goal[0]
                y_diff = y - self.goal[1]

                theta = math.atan2(y_diff, x_diff)
                # print('theta', theta)

                # await robot.turn_in_place(radians(theta), speed=Angle(degrees=15.0)).wait_for_completed()
                # await robot.drive_straight(distance_mm(distance * 10), speed_mmps(25)).wait_for_completed()
                # # await robot.turn_in_place(radians(h), speed=Angle(degrees=50.0)).wait_for_completed()

                for i in range(1):
                    await robot.set_lift_height(1).wait_for_completed()
                    await robot.set_lift_height(0).wait_for_completed()

                print("INFO: goal reached!!!!!!!")
                sleep_time = 2
                for i in range(sleep_time):
                    time.sleep(1)
                    print(f"INFO: sleeping for {sleep_time - i} seconds")
                print("INFO: PF reset!!!!")
                self.particles = Particle.create_random(PARTICLE_COUNT, self.grid)
                # TODO: GUI stuff
                # gui.show_particles(pf.particles)

                last_pose = robot.pose
                curr_pose = last_pose

            last_pose = curr_pose
            curr_pose = robot.pose

            # TODO: add picked up check
            # if robot.is_picked_up:
            if False:
                print("INFO: cozmo picked up!!")
                print("INFO: PF reset!!!!")
                self.particles = Particle.create_random(PARTICLE_COUNT, self.grid)
                # gui.show_particles(self.particles)
                # await robot.say_text("Put me down").wait_for_completed()
                self.last_pose = robot.pose
                self.curr_pose = self.last_pose
                has_been_picked_up = True
        ############################################################################

    def update(self, odom, r_marker_list):
        # ---------- Motion model update ----------
        self.motion_update(self.particles, odom)

        # ---------- Sensor (markers) model update ----------
        self.particles = self.measurement_update(r_marker_list, self.grid)

        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return m_x, m_y, m_h, m_confident

    def motion_update(self, odom):
        """ Particle filter motion update

            Arguments:
            particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                    before motion update
            odom -- noisy odometry measurements, a pair of robot pose, i.e. last time
                    step pose and current time step pose

            Returns: the list of particle representing belief \tilde{p}(x_{t} | u_{t})
                    after motion update
        """
        alpha1 = 0.001
        alpha2 = 0.001
        alpha3 = 0.005
        alpha4 = 0.005

        x_t = odom[1]  # represents pose at time t
        x_t_min1 = odom[0]  # represents pose at time t-1

        s_rot1 = math.degrees(math.atan2(
            x_t[1] - x_t_min1[1], x_t[0] - x_t_min1[0])) - x_t_min1[2]
        s_trans = math.sqrt(
            math.pow(x_t_min1[0] - x_t[0], 2) + math.pow(x_t_min1[1] - x_t[1], 2))
        s_rot2 = x_t[2] - x_t_min1[2] - s_rot1

        for particle in self.particles:
            sb_rot1 = s_rot1 - sample((alpha1 * s_rot1) + (alpha2 * s_trans))
            sb_trans = s_trans - \
                       sample((alpha3 * s_trans) + (alpha4 * (s_rot1 + s_rot2)))
            sb_rot2 = s_rot2 - sample((alpha1 * s_rot2) + (alpha2 * s_trans))
            # x_prime = particle.x + \
            #    (sb_trans * math.cos(math.radians(particle.h + sb_rot1)))
            # y_prime = particle.y + \
            #    (sb_trans * math.sin(math.radians(particle.h + sb_rot1)))
            # h_prime = particle.h + sb_rot1 + sb_rot2

            # particle.x = x_prime
            # particle.y = y_prime
            # particle.h = h_prime
            particle.move(sb_rot1, sb_trans, sb_rot2)

    def measurement_update(self, measured_marker_list, grid):
        """ Particle filter measurement update

        Arguments:
        particles -- a list of particle represents belief \tilde{p}(x_{t} | u_{t})
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
            p_marks = particle.read_markers(grid)
            q_values = []
            # need condition if len(p_marks == 0, but len of robot markers != 0)
            if len(p_marks) == 0:
                q_values = [0]
            # if the particle is not in the grid, ignore
            elif not grid.is_in(particle.x, particle.y):
                q_values = [0]
                npos = grid.random_place()
                particle.x = npos[0]
                particle.y = npos[1]
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
                        robot_r = math.sqrt(math.pow(
                            robot_mark[0], 2) + math.pow(robot_mark[1], 2))
                        robot_theta = math.atan2(robot_mark[1], robot_mark[0])
                        delta_r = r - robot_r
                        delta_theta = theta - robot_theta
                        delta_h = (robot_mark[2] % 360) - (mark[2] % 360)

                        q = normpdf(delta_r, 0, MARKER_TRANS_SIGMA) * \
                            normpdf(math.degrees(delta_theta), 0, MARKER_ROT_SIGMA) * \
                            normpdf(delta_h, 0, MARKER_ROT_SIGMA)
                        q_marker_values.append(q)

                    # find the highest probability (most likely marker seen by particle and robot are the same)
                    # (not sure if this is the right way to do this, verus just multiplying all the deltas)
                    # However, multiplying all the deltas would return a low prob if one marker is very close to another (correctly so) but
                    # Extremely far from a seperate marker (also correctly so, since they are not the same markers)
                    q_values.append(np.prod(q_marker_values))

            # product of each prob on prb list
            particle_probability = np.prod(q_values)
            measured_particles.append(particle)
            weights.append(particle_probability)

        # normalize weights
        weights = np.array(weights)
        weights[np.isnan(weights)] = 0
        sum = np.sum(weights)
        if sum:
            norm_weights = weights / np.sum(weights)
        else:
            # return particles
            norm_weights = weights
            return Particle.create_random(PARTICLE_COUNT, grid)

        return_particles = []
        indices = [np.random.choice(
            np.arange(0, len(measured_particles)), p=norm_weights) for i in range(len(measured_particles))]

        for i in indices:
            return_particles.append(copy.copy(measured_particles[i]))

        # resample new particle (give weights and resample according to weights)
        # measured_particles = list of tuples of particles w/ corresponding probability value
        # time.sleep(1)
        return return_particles


def sample(x):
    return random.gauss(0, x)


def normpdf(x, mean, sd):
    var = float(sd) ** 2
    denom = (2 * math.pi * var) ** .5
    num = math.exp(-(float(x) - float(mean)) ** 2 / (2 * var))
    return num / denom


'''
So you need a radius, phi, and s values as presented (this is going to be similar to the rot1, trans, rot2 from the motion update).
What you should do is get the difference from each and find the probability of being at that difference according to a gaussian pdf
of mean 0 and standard deviation sigma. You can either just use the formula or use a library that implements the formula for you,
it should not make a difference. The prob function would assume a 0 mean, take the x (difference of the feature) and sigma (std deviation of the feature,
or variance depending upon your implementation). The sigma values are in the setting.py file, the ones you would use are
MARKER_TRANS_SIGMA and MARKER_ROT_SIGMA. These are imported into particle_filter.py in the starter code.
'''
