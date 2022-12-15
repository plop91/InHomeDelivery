"""
Camera Calibration
Ian Sodersjerna
"""
import numpy as np
import cv2
import os


def read_chessboards(images):
    """
    Charuco base pose estimation.
    """
    print("POSE ESTIMATION STARTS:")
    all_corners = []
    all_ids = []
    decimator = 0
    # SUB PIXEL CORNER DETECTION CRITERION
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

    if len(images) > 0:
        for im in images:
            print("=> Processing image {0}".format(im))
            frame = cv2.imread(im)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)

            if len(corners) > 0:
                # SUB PIXEL DETECTION
                for corner in corners:
                    cv2.cornerSubPix(gray, corner,
                                     winSize=(3, 3),
                                     zeroZone=(-1, -1),
                                     criteria=criteria)
                res2 = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
                if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3 and decimator % 1 == 0:
                    all_corners.append(res2[1])
                    all_ids.append(res2[2])

            decimator += 1

        imsize = gray.shape
        return all_corners, all_ids, imsize
    return None, None, None


def calibrate_camera(all_corners, all_ids, im_size):
    """
    Calibrates the camera using the dected corners.
    """
    print("CAMERA CALIBRATION")

    camera_matrix_init = np.array([[1000., 0., im_size[0] / 2.],
                                 [0., 1000., im_size[1] / 2.],
                                 [0., 0., 1.]])

    dist_coeffs_init = np.zeros((5, 1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
        charucoCorners=all_corners,
        charucoIds=all_ids,
        board=board,
        imageSize=im_size,
        cameraMatrix=camera_matrix_init,
        distCoeffs=dist_coeffs_init,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors


if __name__ == "__main__":
    """
    Basic test program to generate charuco boards
    """
    import argparse

    parser = argparse.ArgumentParser(description='Create Camera Calibration matrices')

    parser.add_argument('-d', '--dict', type=str, default="DICT_ARUCO_ORIGINAL")
    parser.add_argument('-i', '--input_dir', type=str, default=os.path.join("data", "calibration_images"))

    args = parser.parse_args()

    aruco_dicts = {
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

    aruco_dict_type = aruco_dicts[args.dict]
    aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)

    board = cv2.aruco.CharucoBoard_create(5, 7, 0.03, 0.02, aruco_dict)
    image_paths = []
    for f in os.listdir(args.input_dir):
        if f.endswith(".png"):
            image_paths.append(os.path.join(args.input_dir, f))

    corners, ids, imsize = read_chessboards(image_paths)

    ret, mtx, dist, rvecs, tvecs = calibrate_camera(corners, ids, imsize)

    print(mtx)
    print(dist)

    np.save(os.path.join("data", "calibration_matrix"), mtx)
    np.save(os.path.join("data", "distortion_coefficients"), dist)

    exit(0)
