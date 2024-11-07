#pragma once

#include <opencv2/opencv.hpp>

namespace calib {

    /**
     * Calibrate a new stereo camera system by collecting a number of checkerboard
     * point sets, calibrating, and then viewing the disparity map after calibration.
     */
    void calibrate(
        std::string points_file,
        std::string calibration_file,
        std::string blockmatch_file,
        uint16_t target_point_sets,
        std::string input_pipe,
        std::string output_pipe,
        cv::Size resolution,
        cv::Size board_size,
        double square_size,
        double bucket_percentage,
        double baseline_mm,
        double fps
    );

    /**
     * Review an existing calibration. This will open a video stream, overlay
     * the disparity map, and calculate the depth for a small region of interest
     * from the center of the image. The depth is calculated using the formula:
     *      depth = baseline_mm * focal_length / disparity
     */
    void review(
        std::string calibration_file,
        std::string blockmatch_file,
        std::string input_pipe,
        std::string output_pipe,
        cv::Size resolution,
        double baseline_mm,
        double fps
    );

}