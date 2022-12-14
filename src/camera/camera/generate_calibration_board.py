import cv2
import os


def generate_charuco_board():

    # dir the images will be sent in
    image_dir = os.path.join("data", "boards")

    # ensure data folder exists
    if not os.path.exists("data"):
        os.mkdir("data")

    # ensure marker folder exists
    if not os.path.exists(image_dir):
        os.mkdir(image_dir)

    board = cv2.aruco.CharucoBoard_create(5,  # squares X
                                          7,  # squares Y
                                          0.03,  # square length (normally in meters)
                                          0.02,  # marker length (same units as square length)
                                          aruco_dict)
    imboard = board.draw((2000, 2000))

    filepath = os.path.join(image_dir, f"aruco_board.jpg")

    cv2.imwrite(filepath, imboard)
    print(f"wrote image: {filepath}")


if __name__ == "__main__":
    """
    Basic test program to generate charuco boards
    """
    import argparse

    parser = argparse.ArgumentParser(description='Create Camera Calibration matrices')

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

    generate_charuco_board()
