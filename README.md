# Orthus Stereo Camera Calibration and Rectification GUI and Tuner

Calibrate and tune any stereo camera in an easy-to-use web application.


Calibrate individual cameras using the standard OpenCV calibration method explained in the camera calibration post.

Determine the transformation between the two cameras used in the stereo camera setup.

Using the parameters obtained in the previous steps and the stereoCalibrate method, we determine the transformations applied to both the images for stereo rectification.

Finally, a mapping required to find the undistorted and rectified stereo image pair is obtained using the initUndistortRectifyMap method.

This mapping is applied to the original images to get a rectified undistorted stereo image pair.
