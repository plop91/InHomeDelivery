import cv2
import os


def create_calibration_images():
    cam = cv2.VideoCapture(0)
    cv2.namedWindow("capture calibration images")

    if not os.path.exists("data"):
        os.mkdir("data")

    image_dir = os.path.join("data", "calibration_images")
    if not os.path.exists(image_dir):
        os.mkdir(image_dir)

    img_counter = len(os.listdir(image_dir))

    while True:
        ret, frame = cam.read()
        if not ret:
            print("failed to grab frame")
            break
        cv2.imshow("capture calibration images", frame)

        k = cv2.waitKey(1)
        if k % 256 == 27:
            # ESC pressed
            print("Escape hit, closing...")
            break

        elif k % 256 == 32:
            # SPACE pressed
            img_name = f"calibration_image_{img_counter}.png"
            img_filepath = os.path.join(image_dir, img_name)
            cv2.imwrite(img_filepath, frame)
            print(f"{img_name} written!")
            img_counter += 1

    cam.release()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    create_calibration_images()
