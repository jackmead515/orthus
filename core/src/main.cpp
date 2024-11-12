#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "util.h"
#include "calib.h"
#include "stereo.h"

enum Mode {
    NEW_CALIBRATION = 0,
    MULTI_CALIBRATION = 1,
    REVIEW_CALIBRATION = 2,
    STREAM = 3
};

void multi_calib(
    std::string points_file,
    std::string calibration_file,
    cv::Size board_size,
    cv::Size resolution,
    double square_size,
    double bucket_percentage,
    double rms_threshold,
    int iterations = 16,
    int threads = 8
) {
    std::vector<std::vector<cv::Point2f>> left_object_points;
    std::vector<std::vector<cv::Point2f>> right_object_points;

    cv::FileStorage fs(points_file, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        std::cerr << "invalid points file: " << points_file << std::endl;
        std::exit(1);
    }

    fs["left_object_points"] >> left_object_points;
    fs["right_object_points"] >> right_object_points;
    fs.release();

    if (iterations == 1) {
        stereo::Calibration calibration { 
            stereo::calibrate(
                left_object_points,
                right_object_points,
                board_size,
                resolution,
                square_size,
                bucket_percentage
            )
        };

        if (calibration.reprojection_error > rms_threshold) {
            std::cerr << "calibration failed. reprojection: " << calibration.reprojection_error << 
                " epipolar: " << calibration.epipolar_error << std::endl;
            std::exit(1);
        }

        std::cout << "calibration reprojection: " << calibration.reprojection_error << 
            " epipolar: " << calibration.epipolar_error << std::endl;

        stereo::save_calibration(calibration_file, calibration);

        return;

    }

    auto calibrations = stereo::iter_calibration(
        left_object_points,
        right_object_points,
        board_size,
        resolution,
        square_size,
        bucket_percentage,
        rms_threshold,
        iterations,
        threads
    );

    double avg_re {0};
    for (uint16_t i {0}; i < calibrations.size(); i++) avg_re += calibrations[i].reprojection_error;
    avg_re /= calibrations.size();

    for (uint16_t i {0}; i < calibrations.size(); i++) {
        std::cout << "calibration: " << i << " reprojection: " << calibrations[i].reprojection_error <<
            " epipolar: " << calibrations[i].epipolar_error << std::endl;
    }

    // find the calibration closest to the average rms
    double min_diff = std::numeric_limits<double>::max();
    int avg_calibration_index = 0;
    for (uint16_t i {0}; i < calibrations.size(); i++) {
        double diff = std::abs(calibrations[i].reprojection_error - avg_re);
        if (diff < min_diff) {
            min_diff = diff;
            avg_calibration_index = i;
        }
    }

    stereo::Calibration avg_calibration = calibrations[avg_calibration_index];

    std::cout << "selected calibration reprojection: " << avg_calibration.reprojection_error << std::endl;
    std::cout << "selected calibration epipolar: " << avg_calibration.epipolar_error << std::endl;

    stereo::save_calibration(calibration_file, avg_calibration);
}

int main(int argc, char* argv[]) {

    // the first argument should be a path to a file
    if (argc < 2) {
        std::cerr << "usage: " << argv[0] << " <options_file.yml>" << std::endl;
        std::exit(1);
    }

    // the first argument is the output file
    std::string options_file { argv[1] };

    // check if file exists
    cv::FileStorage fs(options_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "invalid options file: " << options_file << std::endl;
        std::exit(1);
    }

    int mode = 0;
    int target_point_sets = 1000;
    cv::Size board_size(10, 7);
    cv::Size resolution(2560, 720);
    double square_size = 2.3;
    double bucket_percentage = 0.1;
    double fps = 30.0;
    double baseline_mm = 60.0;
    std::string input_pipe { "v4l2src device=/dev/video2 io-mode=2 do-timestamp=true ! image/jpeg,framerate=30/1,width=(int)2560,height=(int)720 ! jpegdec ! videoflip method=rotate-180 ! videoconvert ! appsink" };
    std::string output_pipe { "appsrc ! videoconvert ! videorate ! video/x-raw,framerate=15/1,format=I420 ! avenc_mpeg1video bitrate=300000 maxrate=400000 ! mpegtsmux ! udpsink host=0.0.0.0 port=3131" };

    // read the options
    fs["mode"] >> mode;
    fs["resolution"] >> resolution;

    if (mode == Mode::NEW_CALIBRATION) {
        std::string points_file = "points.yml.gz";
        std::string calibration_file = "calibration.yml.gz";
        std::string blockmatch_file = "block_matching.yml";
        
        fs["points_file"] >> points_file;
        fs["calibration_file"] >> calibration_file;
        fs["blockmatch_file"] >> blockmatch_file;
        fs["input_pipe"] >> input_pipe;
        fs["output_pipe"] >> output_pipe;
        fs["fps"] >> fps;
        fs["target_point_sets"] >> target_point_sets;
        fs["board_size"] >> board_size;
        fs["square_size"] >> square_size;
        fs["bucket_percentage"] >> bucket_percentage;
        fs["baseline_mm"] >> baseline_mm;

        std::cout << "New Calibration Mode" << std::endl;
        std::cout << "points_file: " << points_file << std::endl;
        std::cout << "calibration_file: " << calibration_file << std::endl;
        std::cout << "blockmatch_file: " << blockmatch_file << std::endl;
        std::cout << "target_point_sets: " << target_point_sets << std::endl;
        std::cout << "resolution: " << resolution << std::endl;
        std::cout << "board_size: " << board_size << std::endl;
        std::cout << "square_size: " << square_size << std::endl;
        std::cout << "bucket_percentage: " << bucket_percentage << std::endl;
        std::cout << "baseline_mm: " << baseline_mm << std::endl;
        std::cout << "fps: " << fps << std::endl;

        calib::calibrate(
            points_file,
            calibration_file,
            blockmatch_file,
            target_point_sets,
            input_pipe,
            output_pipe,
            resolution,
            board_size,
            square_size,
            bucket_percentage,
            baseline_mm,
            fps
        );

    } else if (mode == Mode::MULTI_CALIBRATION) {
        std::string points_file = "points.yml.gz";
        std::string calibration_file = "calibration.yml.gz";
        int iterations = 16;
        int threads = 8;
        double rms_threshold = 100.0;

        fs["points_file"] >> points_file;
        fs["calibration_file"] >> calibration_file;
        fs["board_size"] >> board_size;
        fs["square_size"] >> square_size;
        fs["bucket_percentage"] >> bucket_percentage;
        fs["iterations"] >> iterations;
        fs["threads"] >> threads;
        fs["rms_threshold"] >> rms_threshold;

        std::cout << "Iterative Calibration Mode" << std::endl;
        std::cout << "points_file: " << points_file << std::endl;
        std::cout << "calibration_file: " << calibration_file << std::endl;
        std::cout << "board_size: " << board_size << std::endl;
        std::cout << "square_size: " << square_size << std::endl;
        std::cout << "bucket_percentage: " << bucket_percentage << std::endl;
        std::cout << "iterations: " << iterations << std::endl;
        std::cout << "threads: " << threads << std::endl;
        std::cout << "rms_threshold: " << rms_threshold << std::endl;

        multi_calib(
            points_file,
            calibration_file,
            board_size,
            resolution,
            square_size,
            bucket_percentage,
            rms_threshold,
            iterations,
            threads
        );
    } else if (mode == Mode::REVIEW_CALIBRATION) {
        std::string calibration_file = "calibration.yml.gz";
        std::string blockmatch_file = "block_matching.yml";
        
        fs["calibration_file"] >> calibration_file;
        fs["blockmatch_file"] >> blockmatch_file;
        fs["baseline_mm"] >> baseline_mm;
        fs["input_pipe"] >> input_pipe;
        fs["output_pipe"] >> output_pipe;
        fs["resolution"] >> resolution;
        fs["fps"] >> fps;

        std::cout << "Review Calibration Mode" << std::endl;
        std::cout << "calibration_file: " << calibration_file << std::endl;
        std::cout << "blockmatch_file: " << blockmatch_file << std::endl;
        std::cout << "baseline_mm: " << baseline_mm << std::endl;
        std::cout << "resolution: " << resolution << std::endl;
        std::cout << "fps: " << fps << std::endl;

        calib::review(
            calibration_file,
            blockmatch_file,
            input_pipe,
            output_pipe,
            resolution,
            baseline_mm,
            fps
        );

    } else if (mode == Mode::STREAM) { 
        std::string calibration_file = "calibration.yml.gz";
        std::string blockmatch_file = "block_matching.yml";
        
        fs["calibration_file"] >> calibration_file;
        fs["blockmatch_file"] >> blockmatch_file;
        fs["baseline_mm"] >> baseline_mm;
        fs["input_pipe"] >> input_pipe;
        fs["output_pipe"] >> output_pipe;
        fs["resolution"] >> resolution;
        fs["fps"] >> fps;

        std::cout << "Stream Mode" << std::endl;
        std::cout << "calibration_file: " << calibration_file << std::endl;
        std::cout << "blockmatch_file: " << blockmatch_file << std::endl;
        std::cout << "baseline_mm: " << baseline_mm << std::endl;
        std::cout << "resolution: " << resolution << std::endl;
        std::cout << "fps: " << fps << std::endl;

    }

    //auto input_pipe { "/home/jack/Mounts/DiskOne/stereo/zed_calibration/compiled.mp4" };
    //auto input_pipe { "/dev/video2" };
    //auto input_pipe { "videotestsrc pattern=10 ! videoconvert ! appsink" };
    //auto output_pipe { "appsrc stream-type=1 is-live=true ! videoconvert ! video/x-raw,framerate=30/1,format=I420 ! videoconvert ! videorate ! video/x-raw,framerate=15/1 ! avenc_mpeg1video noise-reduction=512 bitrate=100000 ! mpegtsmux ! udpsink host=0.0.0.0 port=3131" };

    return 0;
}