#pragma once

#include <opencv2/opencv.hpp>

namespace calib {

    void calibrate(
        uint16_t target_point_sets,
        std::string video_device,
        std::string output_device,
        cv::Size resolution,
        cv::Size board_size,
        double square_size
    );

}