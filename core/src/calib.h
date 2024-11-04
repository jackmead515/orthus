#pragma once

#include <opencv2/opencv.hpp>

namespace calib {

    void calibrate(
        uint16_t target_point_sets,
        std::string input_pipe,
        std::string output_pipe,
        cv::Size resolution,
        cv::Size board_size,
        double square_size,
        double bucket_percentage
    );

}