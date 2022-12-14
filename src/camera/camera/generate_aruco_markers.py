import os
import cv2
import numpy as np


def generate_aruco_markers(count=10, dict_type="DICT_ARUCO_ORIGINAL"):
    # dict of dicts for arco


    # dir the images will be sent in
    image_dir = os.path.join("data", "markers")

    # ensure data folder exists
    if not os.path.exists("data"):
        os.mkdir("data")

    # ensure marker folder exists
    if not os.path.exists(image_dir):
        os.mkdir(image_dir)

    # generate markers and save them as jpegs
    for i in range(count):
        tag = np.zeros((300, 300, 1), dtype="uint8")
        cv2.aruco.drawMarker(aruco_dict, i, 300, tag, 1)

        filepath = os.path.join(image_dir, f"marker{i}.jpg")

        cv2.imwrite(filepath, tag)
        print(f"wrote image: {filepath}")


if __name__ == "__main__":
    """
    Basic test program to generate old markers
    """
    import argparse

    parser = argparse.ArgumentParser(description='Generate Aruco markers')

    parser.add_argument('-c', '--count', type=int, default=10)
    parser.add_argument('-o', '--output', type=str, default=os.path.join("data", "markers"))
    parser.add_argument('-d', '--dict', type=str, default="DICT_ARUCO_ORIGINAL")

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

    generate_aruco_markers()
