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

    // alpha frame that draws the points captured
    static cv::Mat alpha_frame { cv::Mat() };

    // point sets found
    static uint16_t point_sets_found { 0 };

    // the calibration
    static stereo::Calibration calibration;

    // processing lock
    static std::mutex process_lock;

    // alpha frame lock
    static std::mutex alpha_lock;

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

        // cv::FileStorage fs("points.yml.gz", cv::FileStorage::READ);
        // fs["left_object_points"] >> left_object_points;
        // fs["right_object_points"] >> right_object_points;
        // fs.release();
        // calib::point_sets_found = target_point_sets;

        int find_board_flags { cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK };
        cv::TermCriteria subpix_flags(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 50, 0.001);

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

                    calib::alpha_lock.lock();
                    cv::drawChessboardCorners(calib::alpha_frame(cv::Rect(0, 0, calib::alpha_frame.cols / 2, calib::alpha_frame.rows)), board_size, left_corners, left_found);
                    cv::drawChessboardCorners(calib::alpha_frame(cv::Rect(calib::alpha_frame.cols / 2, 0, calib::alpha_frame.cols / 2, calib::alpha_frame.rows)), board_size, right_corners, right_found);
                    calib::alpha_lock.unlock();

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

            stereo::Calibration calibration { 
                stereo::calibrate(
                    left_object_points,
                    right_object_points,
                    board_size,
                    image_size,
                    square_size,
                    bucket_percentage
                )
            };
            
            stereo::save("calibration.yml.gz", calibration);

            calib::calibration = calibration;
            calib::is_calibrating = false;
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

        calib::alpha_frame = cv::Mat(resolution, CV_8UC3, cv::Scalar(0, 0, 0, 0));

        calib::is_calibrated = true;
        calib::is_calibrating = false;
        calib::quit = true;

        cv::FileStorage fs("calibration.yml.gz", cv::FileStorage::READ);
        fs["undistortion_map_left"] >> calib::calibration.undistortion_map_left;
        fs["rectification_map_left"] >> calib::calibration.rectification_map_left;
        fs["undistortion_map_right"] >> calib::calibration.undistortion_map_right;
        fs["rectification_map_right"] >> calib::calibration.rectification_map_right;
        fs["disp_to_depth_mat"] >> calib::calibration.disp_to_depth_mat;
        fs["left_camera_matrix"] >> calib::calibration.left_camera_matrix;
        fs["rms"] >> calib::calibration.rms;
        fs["proj_mats_left"] >> calib::calibration.proj_mats_left;
        fs.release();
        
        cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(128, 31);
        stereo->setMinDisparity(0);
        stereo->setNumDisparities(128);
        stereo->setTextureThreshold(400);
        stereo->setUniquenessRatio(4);
        stereo->setSpeckleWindowSize(1024);
        stereo->setSpeckleRange(16);
        // stereo->setDisp12MaxDiff(32);
        // stereo->setPreFilterType(1);
        // stereo->setPreFilterSize(17);
        // stereo->setPreFilterCap(16);

        cv::Mat frame;
        cv::Mat left_frame;
        cv::Mat right_frame;
        cv::Mat remapped_left;
        cv::Mat remapped_right;
        cv::Mat remapped_left_gray;
        cv::Mat remapped_right_gray;
        cv::Mat disparity;
        cv::Mat depth;
        cv::Mat display;
        cv::Mat alpha;
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
            if (!calib::is_calibrated && calib::process_lock.try_lock()) {
                frame.copyTo(calib::frame);
                calib::process_lock.unlock();
            }

            if (calib::is_calibrating) {
                left_frame = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
                cv::resize(left_frame, display, output_size);
            
                auto display_str = "Calibrating... ";
                auto text_size = cv::getTextSize(display_str, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
                cv::rectangle(display, cv::Point(8, 8), cv::Point(text_size.width + 12, text_size.height + 12), cv::Scalar(0, 0, 0), cv::FILLED);
                cv::putText(display, display_str, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

            } else if (calib::is_calibrated) {
                left_frame = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
                right_frame = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

                cv::remap(left_frame, remapped_left, calib::calibration.undistortion_map_left, calib::calibration.rectification_map_left, cv::INTER_LINEAR);
                cv::remap(right_frame, remapped_right, calib::calibration.undistortion_map_right, calib::calibration.rectification_map_right, cv::INTER_LINEAR);
                cv::cvtColor(remapped_left, remapped_left_gray, cv::COLOR_BGR2GRAY);
                cv::cvtColor(remapped_right, remapped_right_gray, cv::COLOR_BGR2GRAY);

                stereo->compute(remapped_left_gray, remapped_right_gray, disparity);

                cv::filterSpeckles(disparity, 0, 4096, 64);

                disparity.convertTo(disparity, CV_32F, 1.0f / 16.0f);
                //disparity = (disparity / 16.0) / 128.0;

                //cv::reprojectImageTo3D(disparity, depth, calib::calibration.disp_to_depth_mat);

                double baseline = 6.0;
                double focal_length = calib::calibration.proj_mats_left.at<double>(0, 0);

                depth = baseline * focal_length / (disparity > 0);
                cv::Mat roi = depth(cv::Rect(depth.cols / 2 - 25, depth.rows / 2 - 25, 50, 50)) > 0;
                double avg_depth = cv::mean(roi)[0];

                // double baseline = 60.0;
                // cv::Mat depth = baseline * focal_length / (disparity > 0);
                // cv::Mat roi = depth(cv::Rect(disparity.cols / 2 - 25, disparity.rows / 2 - 25, 50, 50)) > 0;
                // double avg_depth = cv::mean(roi)[0] * 0.001;

                cv::normalize(disparity, disparity, 0, 255, cv::NORM_MINMAX, CV_8U);
                cv::applyColorMap(disparity, disparity, cv::COLORMAP_TURBO);

                cv::rectangle(disparity, cv::Point(disparity.cols / 2 - 25, disparity.rows / 2 - 25), cv::Point(disparity.cols / 2 + 25, disparity.rows / 2 + 25), cv::Scalar(0, 255, 0), 2);

                cv::resize(remapped_left, remapped_left, output_size);
                cv::resize(disparity, disparity, output_size);

                cv::addWeighted(remapped_left, 0.4, disparity, 0.6, 0.0, display);

                // draw 50x50 rect in middle of display
                //cv::rectangle(display, cv::Point(display.cols / 2 - 25, display.rows / 2 - 25), cv::Point(display.cols / 2 + 25, display.rows / 2 + 25), cv::Scalar(0, 255, 0), 2);

                //cv::Mat roi = display(cv::Rect(display.cols / 2 - 25, display.rows / 2 - 25, 50, 50));

                // calculate the depth: baseline_mm * focal_length / disparity[disparity > 0]
                

                double rms = calib::calibration.rms;
                auto display_str = "RMS: " + std::to_string(calib::calibration.rms) + " | Depth (cm): " + std::to_string(avg_depth);
                auto text_size = cv::getTextSize(display_str, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
                cv::rectangle(display, cv::Point(8, 8), cv::Point(text_size.width + 12, text_size.height + 12), cv::Scalar(0, 0, 0), cv::FILLED);
                cv::putText(display, display_str, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

            } else {
                left_frame = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
                cv::resize(left_frame, display, output_size);

                auto display_str = "Calibration Mode: " + std::to_string(calib::point_sets_found) + "/" + std::to_string(target_point_sets);
                auto text_size = cv::getTextSize(display_str, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
                cv::rectangle(display, cv::Point(8, 8), cv::Point(text_size.width + 12, text_size.height + 12), cv::Scalar(0, 0, 0), cv::FILLED);
                cv::putText(display, display_str, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                if (calib::alpha_lock.try_lock()) {
                    alpha = calib::alpha_frame;

                    // resize alpha to 20% of the left_frame
                    cv::resize(alpha, alpha, cv::Size(), 0.1, 0.1);

                    // add alpha to the bottom right corner
                    cv::Mat roi = display(cv::Rect(display.cols - alpha.cols - 10, display.rows - alpha.rows - 10, alpha.cols, alpha.rows));
                    cv::addWeighted(roi, 0.5, alpha, 0.5, 0.0, roi);

                    alpha = cv::Mat();
                    calib::alpha_lock.unlock();
                }

            }

            #ifdef DEBUG_WINDOW // display opencv window for debugging
            cv::imshow("Calibration", display);
            if (cv::waitKey(1) == 27) {
                break;
            }
            #endif

            output << display;

            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

            //fps.update(static_cast<double>(duration.count()) / 1000.0);
            //util::print_progress(calib::point_sets_found, target_point_sets, fps.value);
        }

        cv::destroyAllWindows();
        input.release();
        output.release();

        calib::quit = true;
        processor_thread.join();
    }
}

