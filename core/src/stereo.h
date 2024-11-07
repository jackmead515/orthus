#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>

namespace stereo {

    struct Calibration {
        double rms;
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
    };

    struct BlockMatching {
        int max_speckle_size;
        int max_speckle_diff;
        int block_size;
        int min_disparity;
        int num_disparities;
        int texture_threshold;
        int uniqueness_ratio;
        int speckle_window_size;
        int speckle_range;
        int disp12_max_diff;
        int pre_filter_type;
        int pre_filter_size;
        int pre_filter_cap;
    };

    void save_calibration(std::string file, Calibration& calibration);

    Calibration load_calibration(std::string file);

    BlockMatching load_blockmatching(std::string file);

    void save_blockmatching(std::string file, BlockMatching& blockmatch);

    Calibration calibrate(
        std::vector<std::vector<cv::Point2f>>& left_object_points,
        std::vector<std::vector<cv::Point2f>>& right_object_points,
        cv::Size board_size,
        cv::Size image_size,
        double square_size,
        double bucket_percentage
    );

    std::vector<Calibration> avg_calibration(
        std::vector<std::vector<cv::Point2f>>& left_object_points,
        std::vector<std::vector<cv::Point2f>>& right_object_points,
        cv::Size board_size,
        cv::Size image_size,
        double square_size,
        double bucket_percentage,
        double rms_threshold,
        int iterations,
        int threads
    );

    void calibrate_from_file(
        std::string points_file,
        cv::Size board_size,
        cv::Size image_size,
        double square_size,
        double bucket_percentage
    );
}