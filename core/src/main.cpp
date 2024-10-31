#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "calib.h"

int main() {

    // auto build_info { cv::getBuildInformation() };

    // std::cout << build_info << std::endl;

    // std::exit(0);

    //auto input_pipe = "v4l2src device=0 do-timestamp=true num-buffers=1 ! video/x-h264 ! h264parse ! queue ! rtph264pay config-interval=10 pt=96 ! fakesink"
    //auto input_pipe { "4l2src device=0 ! video/x-raw, framerate=30/1, width=2560, height=720, format=RGB ! videoconvert ! appsink" };

    auto input_pipe { "/home/jack/Mounts/DiskOne/stereo/zed_calibration/compiled.mp4" };
    
    auto output_pipe { "appsrc ! videoconvert ! x264enc noise-reduction=10000 tune=zerolatency byte-stream=true threads=1 ! mpegtsmux ! udpsink host=localhost port=9999" };

    try {
        calib::calibrate(
            5000,
            input_pipe,
            output_pipe,
            cv::Size(2560, 720),
            cv::Size(11, 8),
            3.0
        );
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::exit(1);
    }
    
    return 0;
}