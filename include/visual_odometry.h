/*
Author: Shrinidhi Bhat

Purpose of the file:
    - This file contains the class definition for VisualOdometry class.
    - This class is responsible for running the visual odometry pipeline.
    - It reads the dataset, processes the images, and computes the camera pose.
*/
#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include <filesystem>
#include <opencv2/opencv.hpp>
#include <vector>

class VisualOdometry {
    public:
        VisualOdometry() = default;

        /**
         * @brief Runs the visual odometry process on a given dataset.
         * 
         * @param dataset_path The path to the dataset containing images and calibration data.
         * @return int Returns 0 on success, -1 on failure.
         */
        int run(const std::filesystem::path& dataset_path);
    
    private:
        /**
         * @brief Loads images from the specified directory.
         * 
         * @param image_path Path to the image directory (e.g., "image_0").
         * @return std::vector<std::filesystem::path> A sorted list of image paths.
         */
        std::vector<std::filesystem::path> loadImages(const std::filesystem::path& image_path);
        
        /**
         * @brief Retrieves calibration data from the dataset.
         * 
         * @param dataset_path The base path of the dataset.
         * @param focal The focal length will be stored in this reference.
         * @param pp The principal point will be stored in this reference.
         */
        void getCalibrationData(const std::filesystem::path& dataset_path, double& focal, cv::Point2d& pp);

        /**
         * @brief Tracks the features from one image to another.
         * 
         * @param prevImage The previous image.
         * @param currImage The current image.
         * @param prevFeatures The features in the previous image.
         * @param currFeatures The tracked features in the current image.
         * @param status A vector indicating the status of the tracked features.
         */
        void trackFeatures(const cv::Mat& prevImage, const cv::Mat& currImage, 
                        std::vector<cv::Point2f>& prevFeatures, 
                        std::vector<cv::Point2f>& currFeatures, 
                        std::vector<uchar>& status);

        /**
         * @brief Detects features in a given image.
         * 
         * @param image The image in which to detect features.
         * @param points The points where features are detected.
         */
        void detectFeatures(const cv::Mat& image, std::vector<cv::Point2f>& points);


        /**
         * @brief Computes the absolute scale for the given frame.
         * 
         * @param frame_id The frame number.
         * @return double The scale of the translation between frames.
         */
        double getAbsoluteScale(int frame_id);
};

#endif // VISUAL_ODOMETRY_H