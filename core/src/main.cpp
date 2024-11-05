#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "util.h"
#include "calib.h"
#include "stereo.h"

int main() {

    int target_point_sets = 1000;
    cv::Size board_size(10, 7);
    cv::Size resolution(2560, 720);
    double square_size = 2.3;
    double bucket_percentage = 0.1;

    // cv::FileStorage fs("points.yml.gz", cv::FileStorage::READ);

    // std::vector<std::vector<cv::Point2f>> left_object_points;
    // std::vector<std::vector<cv::Point2f>> right_object_points;

    // fs["left_object_points"] >> left_object_points;
    // fs["right_object_points"] >> right_object_points;

    // fs.release();

    // stereo::avg_calibration(
    //     left_object_points,
    //     right_object_points,
    //     board_size,
    //     resolution,
    //     square_size,
    //     bucket_percentage,
    //     16,
    //     8
    // );

    //auto input_pipe { "/home/jack/Mounts/DiskOne/stereo/zed_calibration/compiled.mp4" };
    //auto input_pipe { "/dev/video2" };
    //auto input_pipe { "videotestsrc pattern=10 ! videoconvert ! appsink" };
    auto input_pipe { "v4l2src device=/dev/video2 io-mode=2 do-timestamp=true ! image/jpeg,framerate=30/1,width=(int)2560,height=(int)720 ! jpegdec ! videoflip method=rotate-180 ! videoconvert ! appsink" };
    
    auto output_pipe { "appsrc ! videoconvert ! videorate ! video/x-raw,framerate=15/1,format=I420 ! avenc_mpeg1video bitrate=200000 maxrate=400000 ! mpegtsmux ! udpsink host=0.0.0.0 port=3131" };
    //auto output_pipe { "appsrc stream-type=1 is-live=true ! videoconvert ! video/x-raw,framerate=30/1,format=I420 ! videoconvert ! videorate ! video/x-raw,framerate=15/1 ! avenc_mpeg1video noise-reduction=512 bitrate=100000 ! mpegtsmux ! udpsink host=0.0.0.0 port=3131" };

    // stereo::calibrate_from_file(
    //     "points.yml.gz",
    //     board_size,
    //     cv::Size(1280, 720),
    //     square_size,
    //     bucket_percentage
    // );

    try {
        calib::calibrate(
            target_point_sets,
            input_pipe,
            output_pipe,
            resolution,
            board_size,
            square_size,
            bucket_percentage
        );
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::exit(1);
    }
    
    return 0;
}