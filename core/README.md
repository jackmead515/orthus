# Core of Orthus

At the core, this is the C++ implementation for all the logic to calibrate the camera, and stream video using any number of features provided.


## Installation

Can help to remove OpenCV if it is already installed

```bash
sudo apt-get remove --purge libopencv* python-opencv opencv*
sudo apt-get remove --purge opencv-data
sudo apt-get autoremove
sudo apt-get autoclean

sudo apt-get update -y && sudo apt-get locate -y

sudo updatedb

locate opencv
# rm -rf any directories that are found left
```

```bash
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio

sudo apt install libfmt-dev

git clone https://github.com/opencv/opencv.git

cmake \
-D BUILD_JAVA=OFF \
-D BUILD_PYTHON=OFF \
-D BUILD_opencv_java=OFF \
-D BUILD_opencv_java_bindings_generator=OFF \
-D BUILD_opencv_js=OFF \
-D BUILD_opencv_js_bindings_generator=OFF \
-D BUILD_opencv_python3=OFF \
-D BUILD_opencv_python_bindings_generator=OFF \
-D BUILD_opencv_python_tests=OFF \
-D BUILD_EXAMPLES=OFF \
-D BUILD_TESTS=OFF \
-D BUILD_PERF_TESTS=OFF \
-D BUILD_opencv_apps=OFF \
-D WITH_ONNX=ON \
-D WITH_GSTREAMER=ON \
-D WITH_V4L=ON \
-D WITH_FFMEG=ON \
../opencv

cmake --build .

sudo make install
```


## Calibration

Calibration mode allows the user to calibrate a stereo camera using a checkerboard. This is usually a one time senario, but should be performed every once in a while if the camera is ever taken out of the mount, bumped in a destructive way, or if the medium is changed (fresh water, salt water, air, etc).

When entering calibration mode, the stream will show the left camera's view with an overlay of how many point sets have been collected while the checkerboard is in view. Once the specified number of point sets have been collected, the stream will end and the calibration will be computed and saved.

## Setup

Create a fake video device with modprove v4l2loopback

```bash
sudo modprobe v4l2loopback video_nr=10 card_label="Orthus" exclusive_caps=1
```

Verify the device was created

```bash
v4l2-ctl --list-devices
```

To get rid of these artifacts we post-process the disparity image with a speckle filter controlled by the speckle_size and speckle_range parameters. speckle_size is the number of pixels below which a disparity blob is dismissed as "speckle." speckle_range controls how close in value disparities must be to be considered part of the same blob. In this case the objects in the scene are relatively large, so we can crank speckle_size up to 1000 pixels: 