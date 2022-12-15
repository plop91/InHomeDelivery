"""
Generate calibration images
Ian Sodersjerna
"""
import cv2
import os


def create_calibration_images():
    """
    Generate calibration images by taking pictures with camera.
    :return: None
    """
    # Start video capture
    cam = cv2.VideoCapture(0)

    # Create named window
    cv2.namedWindow("capture calibration images")

    # Check if data folder exists and if not make it
    if not os.path.exists("data"):
        os.mkdir("data")

    # Check if calibration images folder exists and if not make it
    image_dir = os.path.join("data", "calibration_images")
    if not os.path.exists(image_dir):
        os.mkdir(image_dir)

    # count the number of existing images in the calibration images folder
    img_counter = len(os.listdir(image_dir))

    # run until break
    while True:

        # Read camera image
        ret, frame = cam.read()

        # If no image available the camera is experiencing an error so exit
        if not ret:
            print("failed to grab frame")
            break

        # Show image
        cv2.imshow("capture calibration images", frame)

        # wait one sec for user input
        k = cv2.waitKey(1)

        # if user input was ESC then exit
        if k % 256 == 27:
            break

        # if space pressed, save the image as a calibration image
        elif k % 256 == 32:
            img_name = f"calibration_image_{img_counter}.png"
            img_filepath = os.path.join(image_dir, img_name)
            cv2.imwrite(img_filepath, frame)
            print(f"{img_name} written!")
            img_counter += 1

    # Release the camera
    cam.release()

    # Destroy the window
    cv2.destroyAllWindows()


if __name__ == "__main__":
    create_calibration_images()
