#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>

namespace stereo {

    struct Calibration {
        cv::Mat left_camera_matrix;
        cv::Mat left_dist_coeffs;
        cv::Mat right_camera_matrix;
        cv::Mat right_dist_coeffs;
        cv::Mat R;
        cv::Mat T;
        cv::Mat E;
        cv::Mat F;
        cv::Mat rectification_left;
        cv::Mat rectification_right;
        cv::Mat proj_mats_left;
        cv::Mat proj_mats_right;
        cv::Mat disp_to_depth_mat;
        cv::Mat undistortion_map_left;
        cv::Mat undistortion_map_right;
        cv::Mat rectification_map_left;
        cv::Mat rectification_map_right;

        void save(std::string file);
    };

    Calibration calibrate(
        std::vector<std::vector<cv::Point2f>> left_object_points,
        std::vector<std::vector<cv::Point2f>> right_object_points,
        cv::Size board_size,
        cv::Size image_size,
        double square_size,
        double bucket_percentage
    );

    void calibrate_from_file(
        std::string points_file,
        cv::Size board_size,
        cv::Size image_size,
        double square_size,
        double bucket_percentage
    );
}