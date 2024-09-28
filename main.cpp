/*
Author: Shrinidhi Bhat

Purpose of the file:
    - This file contains the main function.
    - The main function instantiates the VisualOdometry class and runs the visual odometry pipeline.
*/

#include <filesystem>

#include "visual_odometry.h"


int main(int argc, char** argv) {
    // Instantiate the VisualOdometry class
    VisualOdometry visualodometry;

    // Hard code the path to the dataset (Can be made configurable)
    std::filesystem::path kitti_dataset_path = std::filesystem::current_path() / "../dataset" / "data_odometry_gray";
    std::filesystem::path imageset_path = kitti_dataset_path / "dataset" / "sequences" / "08";

    // Run the visual odometry pipeline
    int ret = visualodometry.run(imageset_path);

    return ret;
}