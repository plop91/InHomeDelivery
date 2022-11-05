#!/usr/bin/env python3
import math

import cv2
from numpy.linalg import inv
import threading

from ar_markers.hamming.detect import detect_markers

from gui import GUIWindow
from particle_filter import *
from utils import *

# camera params
camK = np.matrix([[295, 0, 160], [0, 295, 120], [0, 0, 1]], dtype='float32')

# marker size in inches
marker_size = 3.5

# map
Map_filename = "map_arena.json"




async def image_processing(robot):
    global camK, marker_size

    #  TODO: recv the image here
    # event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

    # convert camera image to opencv format
    # TODO: convvert to opencv format
    opencv_image = None
    # opencv_image = np.asarray(event.image)

    # detect markers
    markers = detect_markers(opencv_image, marker_size, camK)

    # show markers
    for marker in markers:
        marker.highlite_marker(opencv_image, draw_frame=True, camK=camK)
        # print("ID =", marker.id);
        # print(marker.contours);
    # cv2.imshow("Markers", opencv_image)

    return markers


# calculate marker pose
def cvt_2d_marker_measurements(ar_markers):
    marker2d_list = []

    for m in ar_markers:
        R_1_2, J = cv2.Rodrigues(m.rvec)
        R_1_1p = np.matrix([[0, 0, 1], [0, -1, 0], [1, 0, 0]])
        R_2_2p = np.matrix([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
        R_2p_1p = np.matmul(np.matmul(inv(R_2_2p), inv(R_1_2)), R_1_1p)
        # print('\n', R_2p_1p)
        yaw = -math.atan2(R_2p_1p[2, 0], R_2p_1p[0, 0])

        x, y = m.tvec[2][0] + 0.5, -m.tvec[0][0]
        # print('x =', x, 'y =', y,'theta =', yaw)

        # remove any duplate markers
        dup_thresh = 2.0
        find_dup = False
        for m2d in marker2d_list:
            if grid_distance(m2d[0], m2d[1], x, y) < dup_thresh:
                find_dup = True
                break
        if not find_dup:
            marker2d_list.append((x, y, math.degrees(yaw)))

    return marker2d_list


# compute robot odometry based on past and current pose
def compute_odometry(curr_pose, cvt_inch=True):
    global last_pose
    last_x, last_y, last_h = last_pose.position.x, last_pose.position.y, \
                             last_pose.rotation.angle_z.degrees
    curr_x, curr_y, curr_h = curr_pose.position.x, curr_pose.position.y, \
                             curr_pose.rotation.angle_z.degrees

    if cvt_inch:
        last_x, last_y = last_x / 25.6, last_y / 25.6
        curr_x, curr_y = curr_x / 25.6, curr_y / 25.6

    return [[last_x, last_y, last_h], [curr_x, curr_y, curr_h]]




if __name__ == '__main__':
    # cozmo thread
    # TODO: investigate running ros node in second thread if possible then do so here.
    # robot_thread = RobotThread()
    # robot_thread.start()

    # init
    grid = CozGrid(Map_filename)
    gui = GUIWindow(grid)

    gui.start()
