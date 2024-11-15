#include <iostream>
#include <chrono>
#include <random>
#include <thread>

#include <opencv2/opencv.hpp>

int random_int(int min, int max) {
    return min + (rand() % (max - min + 1));
}

namespace stereo {

    struct Calibration {
        double reprojection_error;
        double epipolar_error;
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
    };

    struct RectificationMaps {
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

    double compute_epipolar_error(
        std::vector<std::vector<cv::Point2f>>& left_object_points,
        std::vector<std::vector<cv::Point2f>>& right_object_points,
        cv::Mat& left_camera_matrix,
        cv::Mat& left_dist_coeffs,
        cv::Mat& right_camera_matrix,
        cv::Mat& right_dist_coeffs,
        cv::Mat& F
    ) {
        double error { 0.0 };
        int num_points { 0 };

        std::vector<cv::Vec3f> left_lines;
        std::vector<cv::Vec3f> right_lines;

        for (uint16_t i { 0 }; i < left_object_points.size(); i++) {
            int lp = left_object_points[i].size();
            cv::Mat lm(left_object_points[i]);
            cv::Mat rm(right_object_points[i]);

            cv::undistortPoints(lm, lm, left_camera_matrix, left_dist_coeffs, cv::noArray(), left_camera_matrix);
            cv::computeCorrespondEpilines(lm, 1, F, left_lines);

            cv::undistortPoints(rm, rm, right_camera_matrix, right_dist_coeffs, cv::noArray(), right_camera_matrix);
            cv::computeCorrespondEpilines(rm, 2, F, right_lines);

            for (int j { 0 }; j < lp; j++) {
                double err = std::abs(left_object_points[i][j].x * right_lines[j][0] + left_object_points[i][j].y * right_lines[j][1] + right_lines[j][2]) +
                             std::abs(right_object_points[i][j].x * left_lines[j][0] + right_object_points[i][j].y * left_lines[j][1] + left_lines[j][2]);

                error += err;
            }

            num_points += lp;
        }

        auto avg_error = error / num_points;

        return avg_error;
    }

    RectificationMaps compute_rectification_maps(Calibration& calibration, cv::Size image_size) {
        cv::Mat undistortion_map_left;
        cv::Mat undistortion_map_right;
        cv::Mat rectification_map_left;
        cv::Mat rectification_map_right;

        cv::initUndistortRectifyMap(
            calibration.left_camera_matrix,
            calibration.left_dist_coeffs,
            calibration.rectification_left,
            calibration.proj_mats_left,
            image_size,
            CV_32FC1,
            undistortion_map_left,
            rectification_map_left
        );

        cv::initUndistortRectifyMap(
            calibration.right_camera_matrix,
            calibration.right_dist_coeffs,
            calibration.rectification_right,
            calibration.proj_mats_right,
            image_size,
            CV_32FC1,
            undistortion_map_right,
            rectification_map_right
        );

        return RectificationMaps {
            undistortion_map_left,
            undistortion_map_right,
            rectification_map_left,
            rectification_map_right
        };
    }

    void save_calibration(std::string file, Calibration& calibration) {
        cv::FileStorage fs(file, cv::FileStorage::WRITE);
        fs << "reprojection_error" << calibration.reprojection_error;
        fs << "epipolar_error" << calibration.epipolar_error;
        fs << "left_camera_matrix" << calibration.left_camera_matrix;
        fs << "left_dist_coeffs" << calibration.left_dist_coeffs;
        fs << "right_camera_matrix" << calibration.right_camera_matrix;
        fs << "right_dist_coeffs" << calibration.right_dist_coeffs;
        fs << "R" << calibration.R;
        fs << "T" << calibration.T;
        fs << "E" << calibration.E;
        fs << "F" << calibration.F;
        fs << "rectification_left" << calibration.rectification_left;
        fs << "rectification_right" << calibration.rectification_right;
        fs << "proj_mats_left" << calibration.proj_mats_left;
        fs << "proj_mats_right" << calibration.proj_mats_right;
        fs << "disp_to_depth_mat" << calibration.disp_to_depth_mat;
        fs.release();
    }

    Calibration load_calibration(std::string file) {
        cv::FileStorage fs(file, cv::FileStorage::READ);

        if (!fs.isOpened()) {
            throw std::runtime_error("Failed to open calibration file: " + file);
        }

        Calibration calibration;

        fs["reprojection_error"] >> calibration.reprojection_error;
        fs["epipolar_error"] >> calibration.epipolar_error;
        fs["left_camera_matrix"] >> calibration.left_camera_matrix;
        fs["left_dist_coeffs"] >> calibration.left_dist_coeffs;
        fs["right_camera_matrix"] >> calibration.right_camera_matrix;
        fs["right_dist_coeffs"] >> calibration.right_dist_coeffs;
        fs["R"] >> calibration.R;
        fs["T"] >> calibration.T;
        fs["E"] >> calibration.E;
        fs["F"] >> calibration.F;
        fs["rectification_left"] >> calibration.rectification_left;
        fs["rectification_right"] >> calibration.rectification_right;
        fs["proj_mats_left"] >> calibration.proj_mats_left;
        fs["proj_mats_right"] >> calibration.proj_mats_right;
        fs["disp_to_depth_mat"] >> calibration.disp_to_depth_mat;
        fs.release();

        return calibration;
    }

    BlockMatching load_blockmatching(std::string file) {
        cv::FileStorage fs(file, cv::FileStorage::READ);

        if (!fs.isOpened()) {
            throw std::runtime_error("Failed to open block matching file: " + file);
        }

        BlockMatching blockmatch;

        fs["max_speckle_size"] >> blockmatch.max_speckle_size;
        fs["max_speckle_diff"] >> blockmatch.max_speckle_diff;
        fs["block_size"] >> blockmatch.block_size;
        fs["min_disparity"] >> blockmatch.min_disparity;
        fs["num_disparities"] >> blockmatch.num_disparities;
        fs["texture_threshold"] >> blockmatch.texture_threshold;
        fs["uniqueness_ratio"] >> blockmatch.uniqueness_ratio;
        fs["speckle_window_size"] >> blockmatch.speckle_window_size;
        fs["speckle_range"] >> blockmatch.speckle_range;
        fs["disp12_max_diff"] >> blockmatch.disp12_max_diff;
        fs["pre_filter_type"] >> blockmatch.pre_filter_type;
        fs["pre_filter_size"] >> blockmatch.pre_filter_size;
        fs["pre_filter_cap"] >> blockmatch.pre_filter_cap;
        fs.release();

        return blockmatch;
    }

    void save_blockmatching(std::string file, BlockMatching& blockmatch) {
        cv::FileStorage fs(file, cv::FileStorage::WRITE);
        fs << "max_speckle_size" << blockmatch.max_speckle_size;
        fs << "max_speckle_diff" << blockmatch.max_speckle_diff;
        fs << "block_size" << blockmatch.block_size;
        fs << "min_disparity" << blockmatch.min_disparity;
        fs << "num_disparities" << blockmatch.num_disparities;
        fs << "texture_threshold" << blockmatch.texture_threshold;
        fs << "uniqueness_ratio" << blockmatch.uniqueness_ratio;
        fs << "speckle_window_size" << blockmatch.speckle_window_size;
        fs << "speckle_range" << blockmatch.speckle_range;
        fs << "disp12_max_diff" << blockmatch.disp12_max_diff;
        fs << "pre_filter_type" << blockmatch.pre_filter_type;
        fs << "pre_filter_size" << blockmatch.pre_filter_size;
        fs << "pre_filter_cap" << blockmatch.pre_filter_cap;
        fs.release();
    }

    Calibration calibrate(
        std::vector<std::vector<cv::Point2f>>& left_object_points,
        std::vector<std::vector<cv::Point2f>>& right_object_points,
        cv::Size board_size,
        cv::Size image_size,
        double square_size,
        double bucket_percentage
    ) {

        // get a list of the indicies and shuffle them to randomly select points
        std::vector<int> indicies(left_object_points.size());
        for (u_int16_t i {0}; i < left_object_points.size(); i++) indicies.push_back(i);
        std::shuffle(indicies.begin(), indicies.end(), std::random_device());

        // we want to gather images equally distributed over the image plane.
        // So we need to bucketize the image plane into a grid and randomly select points from each bucket.
        // bucket size is based on the percentage of the image plane we want to cover.
        // 1% means we will have 10,000 buckets (100x100) and potentially 10,000 sets of points.
        // 5% means we will have 400 buckets (20x20) and potentially 400 sets of points.
        // 10% means we will have 100 buckets (10x10) and potentially 100 sets of points.

        std::vector<std::vector<cv::Point2f>> left_points;
        std::vector<std::vector<cv::Point2f>> right_points;

        // calculate the size of the bucket based on the percentage of the image plane we want to cover.
        // This will give us a double value. But we want that because out object points are doubles as well.
        double bucket_width = static_cast<double>(image_size.width) * bucket_percentage;
        double bucket_height = static_cast<double>(image_size.height) * bucket_percentage;

        // Calculate the integer percentage out of 100 for the loop.
        // This is to avoid floating point errors.
        int int_perc = static_cast<int>(bucket_percentage * 100);
        
        for (u_int16_t xp { 0 }; xp < 100; xp += int_perc) {
            for (u_int16_t yp { 0 }; yp < 100; yp += int_perc) {
                
                // Calculate the x and y coordinates based on the percentage of the image plane.
                // Again, this produces a double value, but we want that because our object points are doubles.
                double x = (static_cast<double>(xp)/100.0) * static_cast<double>(image_size.width);
                double y = (static_cast<double>(yp)/100.0) * static_cast<double>(image_size.height);

                for (u_int16_t i { 0 }; i < indicies.size(); i++) {
                    int index = indicies[i];
                    cv::Point2f left_point = left_object_points[index][0];

                    // if the point is within the bucekt, we can add the entire point set
                    // and remove that point set from the list of options.
                    if (left_point.x >= x && left_point.x < x + bucket_width && left_point.y >= y && left_point.y < y + bucket_height) {
                        left_points.push_back(left_object_points[index]);
                        right_points.push_back(right_object_points[index]);
                        indicies.erase(indicies.begin() + i);
                        break;
                    }
                }
            }
        }

        // Get the object point coordinates buffer;
        std::vector<std::vector<cv::Point3f>> object_points;
        for (u_int16_t i {0}; i < left_points.size(); i++) {
            std::vector<cv::Point3f> point_coords;
            for (int j {0}; j < board_size.height; j++) {
                for (int i {0}; i < board_size.width; i++) {
                    point_coords.push_back(cv::Point3f((double)i * square_size, (double)j * square_size, 0));
                }
            }
            object_points.push_back(point_coords);
        }

        cv::Mat left_camera_matrix; // camera matrix for left camera
        cv::Mat left_dist_coeffs; // distortion coefficients for left camera
        cv::Mat right_camera_matrix; // camera matrix for right camera
        cv::Mat right_dist_coeffs; // distortion coefficients for right camera
        cv::Mat left_rotation; // rotation matrix specific to left camera
        cv::Mat left_translation; // translation matrix specific to left camera
        cv::Mat right_rotation; // rotation matrix specific to right camera
        cv::Mat right_translation; // translation matrix specific to right camera

        cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5);
        int flags { cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_SAME_FOCAL_LENGTH };
        //int flags { cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_ZERO_TANGENT_DIST };
        //int flags { cv::CALIB_SAME_FOCAL_LENGTH + cv::CALIB_ZERO_TANGENT_DIST };

        left_camera_matrix = initCameraMatrix2D(object_points, left_points, image_size, 0);
        right_camera_matrix = initCameraMatrix2D(object_points, right_points, image_size, 0);

        // double left_rms = cv::calibrateCamera(
        //     object_points,
        //     left_points,
        //     image_size,
        //     left_camera_matrix,
        //     left_dist_coeffs,
        //     left_rotation,
        //     left_translation,
        //     flags
        // );

        // double right_rms = cv::calibrateCamera(
        //     object_points,
        //     right_points,
        //     image_size,
        //     right_camera_matrix,
        //     right_dist_coeffs,
        //     right_rotation,
        //     right_translation,
        //     flags
        // );

        // bool valid_calibration = cv::checkRange(left_camera_matrix) 
        //     && cv::checkRange(left_dist_coeffs) 
        //     && cv::checkRange(right_camera_matrix)
        //     && cv::checkRange(right_dist_coeffs);

        // std::cout << "left_rms: " << left_rms << std::endl;
        // std::cout << "right_rms: " << right_rms << std::endl;
        // std::cout << "valid_calibration: " << valid_calibration << std::endl;
        
        cv::Mat R; // rotational matrix for stereo camera
        cv::Mat T; // translation matrix for stereo camera
        cv::Mat E; // essential matrix for stereo camera
        cv::Mat F; // fundamental matrix for stereo camera

        double reprojection_error = cv::stereoCalibrate(
            object_points,
            left_points,
            right_points,
            left_camera_matrix,
            left_dist_coeffs,
            right_camera_matrix,
            right_dist_coeffs,
            image_size,
            R,
            T,
            E,
            F,
            flags,
            criteria
        );

        double epipolar_error = stereo::compute_epipolar_error(
            left_points,
            right_points,
            left_camera_matrix,
            left_dist_coeffs,
            right_camera_matrix,
            right_dist_coeffs,
            F
        );

        cv::Mat rectification_left;
        cv::Mat rectification_right;
        cv::Mat proj_mats_left;
        cv::Mat proj_mats_right;
        cv::Mat disp_to_depth_mat;

        cv::stereoRectify(
            left_camera_matrix,
            left_dist_coeffs,
            right_camera_matrix,
            right_dist_coeffs,
            image_size,
            R,
            T,
            rectification_left,
            rectification_right,
            proj_mats_left,
            proj_mats_right,
            disp_to_depth_mat,
            cv::CALIB_ZERO_DISPARITY
        );

        return Calibration {
            reprojection_error,
            epipolar_error,
            left_camera_matrix,
            left_dist_coeffs,
            right_camera_matrix,
            right_dist_coeffs,
            R,
            T,
            E,
            F,
            rectification_left,
            rectification_right,
            proj_mats_left,
            proj_mats_right,
            disp_to_depth_mat
        };
    }

    std::vector<Calibration> iter_calibration(
        std::vector<std::vector<cv::Point2f>>& left_object_points,
        std::vector<std::vector<cv::Point2f>>& right_object_points,
        cv::Size board_size,
        cv::Size image_size,
        double square_size,
        double bucket_percentage,
        double rms_threshold,
        int iterations,
        int threads
    ) {
        std::vector<Calibration> calibrations;
        std::mutex lock;

        int completed = 0;

        do {

            int amount = iterations - completed;
            if (amount > threads) amount = threads;

            std::vector<std::thread> thread_pool;

            for (int i {0}; i < amount; i++) {
                thread_pool.push_back(std::thread([&] {
                    Calibration calibration { calibrate(left_object_points, right_object_points, board_size, image_size, square_size, bucket_percentage) };

                    if (calibration.reprojection_error > rms_threshold) return;

                    lock.lock();
                    calibrations.push_back(calibration);
                    lock.unlock();
                }));
            }

            for (int i {0}; i < threads; i++) thread_pool[i].join();

            completed += amount;

        } while (completed < iterations);

        return calibrations;
    }

    void calibrate_from_file(
        std::string points_file,
        cv::Size board_size,
        cv::Size image_size,
        double square_size,
        double bucket_percentage
    ) {
        cv::FileStorage fs(points_file, cv::FileStorage::READ);

        std::vector<std::vector<cv::Point2f>> left_object_points;
        std::vector<std::vector<cv::Point2f>> right_object_points;

        fs["left_object_points"] >> left_object_points;
        fs["right_object_points"] >> right_object_points;

        fs.release();

        Calibration calibration { calibrate(left_object_points, right_object_points, board_size, image_size, square_size, bucket_percentage) };

        save_calibration("calibration.yml.gz", calibration);
    }
}