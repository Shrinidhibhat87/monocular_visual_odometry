/*
Author: Shrinidhi Bhat https://stackoverflow.com/tags

Purpose of the file:
    - Source file for the implementation of the odometry class
*/

#include "visual_odometry.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d.hpp"

#include <iostream>
#include <iterator> // for ostream_iterator
#include <vector>
#include <fstream>
#include <filesystem>

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000


// Function to get absolute scale
double VisualOdometry::getAbsoluteScale(int frame_id){
  
  std::string line;
  int i = 0;

  // Define the path to the file
  std::filesystem::path dataset_path = std::filesystem::current_path() / "../dataset" / "data_odometry_poses/dataset/poses/08.txt";
  // Open the file
  std::ifstream myfile(dataset_path.string());
  
  // Initialize the variables to store the x, y, z values
  double x =0, y=0, z = 0;
  double x_prev, y_prev, z_prev;

  // Read the file line by line
  if (myfile.is_open())
  {
    while ((getline(myfile,line) ) && (i<=frame_id))
    {
      z_prev = z;
      x_prev = x;
      y_prev = y;

      // Read the line
      std::istringstream in(line);

      for (int j=0; j<12; j++)  {
        in >> z ;
        if (j==7) y=z;
        if (j==3)  x=z;
      }
      
      i++;
    }
    myfile.close();
  }

  else {
    std::cout << "Unable to open file";
    return 0;
  }

  return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;

}

std::vector<std::filesystem::path> VisualOdometry::loadImages(const std::filesystem::path& image_path) {
    std::vector<std::filesystem::path> image_files;
    for (const auto& entry : std::filesystem::directory_iterator(image_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".png") {
            image_files.push_back(entry.path());
        }
    }
    return image_files;
}

// The below function gets the calibration data from the file.
// The input to the function is the dataset path and the output is the focal length and principal point.
void VisualOdometry::getCalibrationData(const std::filesystem::path& dataset_path, double& focal, cv::Point2d& pp) {
    // Read the calibration file
    std::filesystem::path calib_file = dataset_path / "calib.txt";

    // Open the file
    std::ifstream calib_file_stream(calib_file);

    // Check if the file was opened correctly
    if (!calib_file_stream.is_open()) {
        std::cerr << "Error!! Could not open the calibration file." << std::endl;
        return;
    }

    // Store a variable for line
    std::string line;

    // Read only the first line of the file
    if (std::getline(calib_file_stream, line)) {
        // Split the line by spaces
        
        std::istringstream iss(line);
        std::vector<std::string> data((std::istream_iterator<std::string>(iss)), std::istream_iterator<std::string>());

        // Extract the focal length and principal point
        focal = std::stod(data[1]); // Focal length: Second element
        pp.x = std::stod(data[3]); // Principal point x coordinate: Fourth element
        pp.y = std::stod(data[7]); // Principal point y coordinate: Eighth element
    } else {
        std::cerr << "Error! Calibration file empty." << std::endl;
    }

    // Close the file
    calib_file_stream.close();
}

// The below function is to detect features in the Image.
// The input to the function is the image. The output is the keypoints.
void VisualOdometry::detectFeatures(const cv::Mat& image, std::vector<cv::Point2f>& points) {

    // The current method to detect features use the FAST algorithm
    // Link: https://docs.opencv.org/3.4/df/d0c/tutorial_py_fast.html

    // Create a FAST feature detector object
    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();

    // Detect keypoints using the FAST algorithm
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(image, keypoints);

    // Convert keypoints to cv::Point2f format
    points.clear();
    for (const auto& keypoint : keypoints) {
        points.push_back(keypoint.pt);
    }

}

// The below function is to track features in the Image.
// The input to the function are the images and the keypoints from the previous image.
// The output is the keypoints in the current image and the status.
void VisualOdometry::trackFeatures(const cv::Mat& prevImage, const cv::Mat& currImage, 
                                   std::vector<cv::Point2f>& prevFeatures, 
                                   std::vector<cv::Point2f>& currFeatures, 
                                   std::vector<uchar>& status) {
    // Parameters for the optical flow algorithm
    // Instantiate a termination criteria object
    // The constructor i count (max number of iterations) and epsilon (minimum error)
    // Link: https://docs.opencv.org/4.x/d9/d5d/classcv_1_1TermCriteria.html
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 20, 0.03);
    cv::Size subPixWinSize(10,10), winSize(31,31);

    // Find the features in the first image
    cv::cornerSubPix(prevImage, prevFeatures, subPixWinSize, cv::Size(-1,-1), termcrit);

    // Calculate the optical flow using the Lucas-Kanade method
    std::vector<float> err;
    // Link: https://docs.opencv.org/3.4/dc/d6b/group__video__track.html#ga473e4b886d0bcc6b65831eb88ed93323
    cv::calcOpticalFlowPyrLK(prevImage, currImage, prevFeatures, currFeatures, status, err, winSize, 3, termcrit, 0, 0.001);

    // Get rid of points for which the KLT tracking failed or those who have gone outside the frame
    int indexCorrection = 0;
    for( int i=0; i<status.size(); i++) {
        cv::Point2f pt = currFeatures.at(i- indexCorrection);
        if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0)) {
            if((pt.x<0)||(pt.y<0)) {
                status.at(i) = 0;
            }
            prevFeatures.erase (prevFeatures.begin() + (i - indexCorrection));
            currFeatures.erase (currFeatures.begin() + (i - indexCorrection));
            indexCorrection++;
        }
    }
}

int VisualOdometry::run(const std::filesystem::path& dataset_path) {

    // Inititalize matrices to get the image
    cv::Mat image_1, image_2;
    cv::Mat rotation_final, translation_final; // Final rotation matrix and translation vector containing the camera pose

    // Initialize value for the scale
    double scale = 1.00;

    // From dataset path, get the imageset path
    // Since we are working with monocular odometry, consider only image_0 or left camera images
    std::filesystem::path image_path = dataset_path / "image_0";

    // Load images using the modular function
    std::vector<std::filesystem::path> image_files = loadImages(image_path);

    // If the number of images are less than 2, return an error
    if (image_files.size() < 2) {
        std::cerr << "Error: Insufficient number of images in the dataset." << std::endl;
        return -1;
    }

    // Sort the image files based on the name
    std::sort(image_files.begin(), image_files.end());

    // Get the first two images
    std::string filename1 = image_files[0].string();
    std::string filename2 = image_files[1].string();

    char text[100];
    int fontFace = cv::FONT_HERSHEY_PLAIN;
    double fontScale = 1;
    int thickness = 1;
    cv::Point textOrg(10, 50);

    // Read the first two images and convert to grayscale
    cv::cvtColor(cv::imread(filename1), image_1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(cv::imread(filename2), image_2, cv::COLOR_BGR2GRAY);

    // Initialize the points used for feature detection and tracking
    std::vector<cv::Point2f> points_1, points_2; // Points to store the keypoints

    // Detect features in the first image
    detectFeatures(image_1, points_1);

    // Track the features in the second image
    std::vector<uchar> status;
    trackFeatures(image_1, image_2, points_1, points_2, status);

    // Get the calibration information
    double focal;
    cv::Point2d pp;
    getCalibrationData(dataset_path, focal, pp);

    // Recover the pose and essential matrix
    cv::Mat Essential, Rotational, translation, mask;

    // Compute the essential matrix using OpenCV
    Essential = cv::findEssentialMat(points_2, points_1, focal, pp, cv::RANSAC, 0.999, 1.0, mask);

    // Recover the pose from the essential matrix
    cv::recoverPose(Essential, points_2, points_1, Rotational, translation, focal, pp, mask);

    // Initialize variables for the current image
    cv::Mat prevImage = image_2;
    cv::Mat currImage;
    std::vector<cv::Point2f> prevFeatures = points_2;
    std::vector<cv::Point2f> currFeatures;

    char filename_c[100];

    // Deep copy the rotation and translation matrices
    rotation_final = Rotational.clone();
    translation_final = translation.clone();

    // Initialize the trajectory image
    cv::Mat traj = cv::Mat::zeros(600, 600, CV_8UC3); // 8-bit unsigned integer matrix with 3 channels

    // Loop over the frames
    for (int num_frame = 2; num_frame < MAX_FRAME; num_frame++) {
        sprintf(filename_c, "%s/%06d.png", image_path.c_str(), num_frame);

        // Get the current image and convert it to grayscale
        cv::Mat color_currImage = cv::imread(filename_c);
        if (color_currImage.empty()) {
            std::cerr << "Error: Could not load image at path: " << filename_c << std::endl;
            break;  // Or handle the error appropriately
        }

        cv::cvtColor(color_currImage, currImage, cv::COLOR_BGR2GRAY);

        // Track features from the previous image to the current image
        trackFeatures(prevImage, currImage, prevFeatures, currFeatures, status);

        // Compute the essential matrix and recover the pose
        Essential = cv::findEssentialMat(currFeatures, prevFeatures, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
        cv::recoverPose(Essential, currFeatures, prevFeatures, Rotational, translation, focal, pp, mask);

        // Get the absolute scale
        scale = getAbsoluteScale(num_frame);

        // Update rotation and translation matrices if scale is valid
        if ((scale > 0.1) && (translation.at<double>(2) > translation.at<double>(0)) && (translation.at<double>(2) > translation.at<double>(1))) {
            translation_final = translation_final + scale * (rotation_final * translation); // Update translation
            rotation_final = Rotational * rotation_final; // Update rotation
        }

        // If the number of features being tracked is below the threshold, re-detect and re-track features
        if (prevFeatures.size() < MIN_NUM_FEAT) {
            detectFeatures(prevImage, prevFeatures);
            trackFeatures(prevImage, currImage, prevFeatures, currFeatures, status);
        }

        // Update the previous image and features
        prevImage = currImage.clone();
        prevFeatures = currFeatures;

        int x = int(translation_final.at<double>(0)) + 300; // x-coordinate offset 
        int y = int(-1 * translation_final.at<double>(2)) + 500; // y-coordinate offset

        // Draw the trajectory
        cv::circle(traj, cv::Point(x, y), 1, CV_RGB(255, 0, 0), 2);

        // Draw a rectangle to show the current location
        cv::rectangle(traj, cv::Point(10, 30), cv::Point(550, 50), CV_RGB(0, 0, 0), cv::FILLED);

        // Display the text
        sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", translation_final.at<double>(0), translation_final.at<double>(1), translation_final.at<double>(2));
        cv::putText(traj, text, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);

        // Display the images
        cv::imshow("Road facing camera", currImage);
        cv::imshow("Trajectory", traj);

        // Wait for 1ms
        cv::waitKey(1);
    }

    return 0;
}
