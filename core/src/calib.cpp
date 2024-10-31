#include <iostream>
#include <chrono>
#include <thread>

#include <opencv2/opencv.hpp>

#include "util.h"

namespace calib {

    struct FPS {
        int fps_buffer_len { 32 };

        int fps_index { 0 };

        double value { 0.0 };

        double fps_buffer[32];

        bool ready { false };

        // initialize the FPS buffer. Fills the buffer with 0.0
        FPS() {
            for (int i { 0 }; i < fps_buffer_len; i++) {
                fps_buffer[i] = 0.0;
            }
        }

        // update the FPS value if the fps_index has reached the buffer length
        void update(double duration) {
            fps_buffer[fps_index] = duration;
            fps_index++;
            if (fps_index >= fps_buffer_len) {
                fps_index = 0;
                double sum_sec = 0.0;
                for (int i { 0 }; i < fps_buffer_len; i++) {
                    sum_sec += fps_buffer[i];
                }

                value = static_cast<double>(fps_buffer_len) / sum_sec;
            }
        }
    };

    // quit flag
    static bool quit { false };

    // frame to process
    static cv::Mat frame { cv::Mat() };

    // point sets found
    static uint16_t point_sets_found { 0 };

    // processing lock
    static std::mutex process_lock;


    // The processor function is launched in another thread in order to process
    // the frames to discover the chessboard corners. Once all the corners are
    // discovered, then the calibration is performed and the thread exits.
    void processor(uint16_t target_point_sets, cv::Size board_size) {

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
            if (calib::quit) {
                break;
            }

            calib::process_lock.lock();

            if (calib::frame.empty()) {
                calib::process_lock.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            
            left_frame = calib::frame(cv::Rect(0, 0, calib::frame.cols / 2, calib::frame.rows));
            right_frame = calib::frame(cv::Rect(calib::frame.cols / 2, 0, calib::frame.cols / 2, calib::frame.rows));

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

            // empty the frame and release the lock
            calib::frame = cv::Mat();
            calib::process_lock.unlock();
        }
    }


    void calibrate(
        uint16_t target_point_sets,
        std::string video_device,
        std::string output_device,
        cv::Size resolution,
        cv::Size board_size,
        double square_size
    ) {
        if (target_point_sets >= 10000) {
            throw std::invalid_argument("target_point_sets must be less than 10000");
        }

        if (resolution.width != 2560 && resolution.height != 720) {
            throw std::invalid_argument("resolution must be 2560x720");
        }

        if ((board_size.width != 9 && board_size.height != 6) && (board_size.width != 11 && board_size.height != 8)) {
            throw std::invalid_argument("board_size must be 9x6 or 11x8");
        }

        if (square_size <= 0.0 || square_size >= 10.0) {
            throw std::invalid_argument("square_size must be greater than 0 and less than 10");
        }

        cv::VideoCapture input(video_device);

        if (!input.isOpened()) {
            throw std::runtime_error("Failed to open input device: " + video_device);
        }

        cv::VideoWriter output(output_device, cv::CAP_GSTREAMER, 0, 30.0, cv::Size(640, 360), true);

        if (!output.isOpened()) {
            throw std::runtime_error("Failed to open output device: " + output_device);
        }

        cv::Mat frame;
        cv::Mat left_frame;
        calib::FPS fps;

        auto processor_thread = std::thread{ processor, target_point_sets, board_size };

        while (true) {
            auto start_time = std::chrono::high_resolution_clock::now();
            
            input >> frame;

            if (frame.empty()) {
                break;
            }

            // send frame to processor if processing lock is available
            // if not, skip the frame and try again with the next frame
            if (calib::process_lock.try_lock()) {
                calib::frame = frame;
                calib::process_lock.unlock();
            }

            left_frame = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));

            // resize in half based on percent
            cv::resize(left_frame, left_frame, cv::Size(), 0.5, 0.5);

            // draw total point sets found
            cv::putText(left_frame, std::to_string(calib::point_sets_found), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

            //cv::imshow("Calibration", left_frame);
            // if (cv::waitKey(1) == 27) {
            //     break;
            // }

            output << left_frame;

            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

            fps.update(static_cast<double>(duration.count()) / 1000.0);
            util::print_progress(calib::point_sets_found, target_point_sets, fps.value);

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

