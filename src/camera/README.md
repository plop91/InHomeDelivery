# Camera
Ian Sodersjerna  
11/2/2022

## Usage

1. Generate Aruco markers  
These markers should be printed on white paper, the size of each marker should be exactly 1 inch, I achieved this by placing the generated images into a Google doc and using their sizing tools to make it print as a one-inch image on an 8.5x11 sheet of paper.
```shell
# input at command line
python generate_aruco_markers.py
# output -> images generated as jpegs in data/markers
```
2. Generate Aruco board  
This board should be printed on an 8.5 sheet of paper each cell should be 1 inch square.
```shell
# input at command line
python generate_calibration_board.py
# output -> image generated as jpeg in data/calibration_boards
```
3. take calibration images  
Take images of the calibration board using desired webcam, the images should have the board at a variety of angles and distances relative to the camera but all images should have a calibration board in frame, approx 20 images is the minimum recommended.
```shell
# input at command line
python generate_cal_images.py
# output -> images generated as jpegs in data/calibration_images
```
4. Generate calibration matrices  
Generate calibration matrices used by aruco to  determine poses of markers.
```shell
# input at command line
python camera_calibration.py
# output -> calibration_matrix.npy and distortion_coefficients.npy
```

## Goal


## Description