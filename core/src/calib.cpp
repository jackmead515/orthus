#include <iostream>
#include <chrono>
#include <thread>

#include <opencv2/opencv.hpp>

#include "util.h"
#include "stereo.h"

// display opencv window for debugging
// #define DEBUG_WINDOW

namespace calib {

    // quit flag to indicate the calibration should stop
    static bool quit { false };

    // flag to indicate if calibration is in progress
    static bool is_calibrating { false };

    // flag to indicate if calibration is complete
    static bool is_calibrated { false };

    // frame to process
    static cv::Mat frame { cv::Mat() };

    // point sets found
    static uint16_t point_sets_found { 0 };

    // processing lock
    static std::mutex process_lock;

    // The processor function is launched in another thread in order to process
    // the frames to discover the chessboard corners. Once all the corners are
    // discovered, then the calibration is performed and the thread exits.
    void processor(
        uint16_t target_point_sets,
        cv::Size resolution,
        cv::Size board_size,
        double square_size,
        double bucket_percentage
    ) {
        cv::Mat left_frame;
        cv::Mat right_frame;
        cv::Mat left_frame_gray;
        cv::Mat right_frame_gray;

        // setup buffers for chessboard corners
        std::vector<cv::Point2f> left_corners(board_size.width * board_size.height);
        std::vector<cv::Point2f> right_corners(board_size.width * board_size.height);

        // setup buffers for object points
        std::vector<std::vector<cv::Point2f>> left_object_points;
        std::vector<std::vector<cv::Point2f>> right_object_points;
        left_object_points.reserve(target_point_sets);
        right_object_points.reserve(target_point_sets);

        int find_board_flags { cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK };
        cv::TermCriteria subpix_flags(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01);

        while (true) {
            if (calib::quit || calib::point_sets_found >= target_point_sets) {
                break;
            }

            calib::process_lock.lock();

            if (calib::frame.empty()) {
                calib::process_lock.unlock();
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            }

            // copy the frame, empty the shared one, and release the lock
            cv::Mat frame { calib::frame };
            calib::frame.data = nullptr;
            calib::process_lock.unlock();
            
            left_frame = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
            right_frame = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

            if (left_frame_gray.empty()) {
                left_frame_gray = cv::Mat(left_frame.size(), CV_8UC1);
                right_frame_gray = cv::Mat(right_frame.size(), CV_8UC1);
            }

            auto left_found = cv::findChessboardCorners(left_frame, board_size, left_corners, find_board_flags);

            if (left_found) {

                auto right_found = cv::findChessboardCorners(right_frame, board_size, right_corners, find_board_flags);

                if (right_found) {
                    cv::cvtColor(left_frame, left_frame_gray, cv::COLOR_BGR2GRAY);
                    cv::cvtColor(right_frame, right_frame_gray, cv::COLOR_BGR2GRAY);

                    cv::cornerSubPix(left_frame_gray, left_corners, cv::Size(11, 11), cv::Size(-1, -1), subpix_flags);
                    cv::cornerSubPix(right_frame_gray, right_corners, cv::Size(11, 11), cv::Size(-1, -1), subpix_flags);

                    left_object_points.push_back(left_corners);
                    right_object_points.push_back(right_corners);

                    calib::point_sets_found += 1;
                }
            }
        }

        if (calib::point_sets_found >= target_point_sets) {

            calib::is_calibrating = true;

            cv::FileStorage fs("points.yml.gz", cv::FileStorage::WRITE);
            fs << "left_object_points" << left_object_points;
            fs << "right_object_points" << right_object_points;
            fs.release();

            cv::Size image_size { cv::Size(resolution.width / 2, resolution.height) };

            stereo::calibrate(
                left_object_points,
                right_object_points,
                board_size,
                image_size,
                square_size,
                bucket_percentage
            );

            calib::is_calibrated = true;
        }
    }

    void calibrate(
        uint16_t target_point_sets,
        std::string input_pipe,
        std::string output_pipe,
        cv::Size resolution,
        cv::Size board_size,
        double square_size,
        double bucket_percentage
    ) {
        if (target_point_sets >= 10000) {
            throw std::invalid_argument("target_point_sets must be less than 10000");
        }

        if (resolution.width != 2560 && resolution.height != 720) {
            throw std::invalid_argument("resolution must be 2560x720");
        }

        if (square_size <= 0.0 || square_size >= 10.0) {
            throw std::invalid_argument("square_size must be greater than 0 and less than 10");
        }

        cv::VideoCapture input(input_pipe, cv::CAP_GSTREAMER);

        if (!input.isOpened()) {
            throw std::runtime_error("Failed to open input device: " + input_pipe);
        }

        std::cout << "Input device: '" << input_pipe  << "' opened" << std::endl;

        int codec = 0;
        cv::Size output_size { resolution.width / 4, resolution.height / 2 };
        cv::VideoWriter output(output_pipe, cv::CAP_GSTREAMER, codec, 30.0, output_size, true);

        if (!output.isOpened()) {
            throw std::runtime_error("Failed to open output device: " + output_pipe);
        }

        std::cout << "Output device: '" << output_pipe  << "' opened" << std::endl;

        cv::Mat undistortion_map_left;
        cv::Mat rectification_map_left;
        cv::Mat undistortion_map_right;
        cv::Mat rectification_map_right;
        cv::Mat disparity_to_depth_map;
        cv::FileStorage fs("calibration.yml.gz", cv::FileStorage::READ);
        fs["undistortion_map_left"] >> undistortion_map_left;
        fs["rectification_map_left"] >> rectification_map_left;
        fs["undistortion_map_right"] >> undistortion_map_right;
        fs["rectification_map_right"] >> rectification_map_right;
        fs["disp_to_depth_mat"] >> disparity_to_depth_map;
        fs.release();
        cv::Mat right_frame;
        cv::Mat remapped_left;
        cv::Mat remapped_right;
        cv::Mat disparity;
        cv::Mat disparity_color;
        cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(48, 45);
        stereo->setMinDisparity(5);
        stereo->setNumDisparities(48);
        stereo->setTextureThreshold(385);
        stereo->setUniquenessRatio(3);
        stereo->setSpeckleWindowSize(2);
        stereo->setSpeckleRange(23);
        stereo->setDisp12MaxDiff(5);
        stereo->setPreFilterType(1);
        stereo->setPreFilterSize(17);
        stereo->setPreFilterCap(16);

        cv::Mat frame;
        cv::Mat left_frame;
        util::FPS fps;
    
        auto processor_thread = std::thread{ processor, target_point_sets, resolution, board_size, square_size, bucket_percentage };

        while (true) {
            auto start_time = std::chrono::high_resolution_clock::now();
            
            input >> frame;

            if (frame.empty()) {
                break;
            }

            // send frame to processor if processing lock is available
            // if not, skip the frame and try again with the next frame
            // if (calib::process_lock.try_lock()) {
            //     calib::frame = frame;
            //     calib::process_lock.unlock();
            // }

            left_frame = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
            
            right_frame = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
            cv::remap(left_frame, remapped_left, undistortion_map_left, rectification_map_left, cv::INTER_LINEAR);
            cv::remap(right_frame, remapped_right, undistortion_map_right, rectification_map_right, cv::INTER_LINEAR);
            cv::cvtColor(remapped_left, remapped_left, cv::COLOR_BGR2GRAY);
            cv::cvtColor(remapped_right, remapped_right, cv::COLOR_BGR2GRAY);

            stereo->compute(remapped_left, remapped_right, disparity);

            cv::filterSpeckles(disparity, 0, 4000, 55);
            cv::normalize(disparity, disparity, 0, 255, cv::NORM_MINMAX, CV_8U);
            cv::applyColorMap(disparity, disparity_color, cv::COLORMAP_JET);

            cv::resize(disparity_color, disparity_color, cv::Size(), 0.5, 0.5);

            // // draw text for calibration mode
            // // resize in half based on percent
            // cv::resize(left_frame, left_frame, output_size);

            // if (calib::is_calibrating) {
            //     auto display_str = "Calibrating... ";
            //     auto text_size = cv::getTextSize(display_str, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
            //     cv::rectangle(left_frame, cv::Point(8, 8), cv::Point(text_size.width + 12, text_size.height + 12), cv::Scalar(0, 0, 0), cv::FILLED);
            //     cv::putText(left_frame, display_str, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

            // } else if (calib::is_calibrated) {
            //     auto display_str = "Calibrated";
            //     auto text_size = cv::getTextSize(display_str, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
            //     cv::rectangle(left_frame, cv::Point(8, 8), cv::Point(text_size.width + 12, text_size.height + 12), cv::Scalar(0, 0, 0), cv::FILLED);
            //     cv::putText(left_frame, display_str, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

            // } else {
            //     auto display_str = "Calibration Mode: " + std::to_string(calib::point_sets_found) + "/" + std::to_string(target_point_sets);
            //     auto text_size = cv::getTextSize(display_str, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
            //     cv::rectangle(left_frame, cv::Point(8, 8), cv::Point(text_size.width + 12, text_size.height + 12), cv::Scalar(0, 0, 0), cv::FILLED);
            //     cv::putText(left_frame, display_str, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

            // }

            #ifdef DEBUG_WINDOW // display opencv window for debugging
            cv::imshow("Calibration", left_frame);
            if (cv::waitKey(1) == 27) {
                break;
            }
            #endif

            output << disparity_color;

            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

            //fps.update(static_cast<double>(duration.count()) / 1000.0);
            //util::print_progress(calib::point_sets_found, target_point_sets, fps.value);

            if (calib::point_sets_found >= target_point_sets) {
                break;
            }

        }

        cv::destroyAllWindows();
        input.release();
        output.release();

        calib::quit = true;
        processor_thread.join();
    }
}

