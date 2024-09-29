# Monocular Odometry Tutorial

This repository provides a tutorial on monocular odometry, a technique used in computer vision to estimate the motion of a camera using a single image sequence. The tutorial aims to help you understand the fundamentals of monocular odometry and provide a step-by-step guide to running the binary included in this repository.

## Table of Contents
- [Introduction](#introduction)
- [Theory](#theory)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Introduction
Monocular odometry is a crucial component in various applications such as robotics, augmented reality, and autonomous vehicles. This tutorial will guide you through the process of understanding and implementing monocular odometry using a single camera for the KITTI dataset.

## Theory

### What is Visual Odometry?
Visual odometry is the process of using camera feeds to estimate how an object or camera moves through space. It primarily works by analyzing keypoints between consecutive frames to infer motion. Visual odometry can be used in conjunction with other forms of odometry, such as wheel odometry, to improve localization accuracy. It helps determine movement patterns, such as whether the object is moving forward, backward, or rotating.

In the case of monocular visual odometry, only a single camera is used. However, this introduces the limitation that motion can only be recovered up to an unknown scale (i.e., the distance moved is relative but not absolute). In contrast, stereo visual odometry, using two cameras, can triangulate features and recover the full motion of an object, including the correct scale.

### Why Do We Need Visual Odometry?
Visual odometry is essential for a wide range of applications, particularly in robotics and autonomous systems, where accurate movement estimation is critical. By estimating the incremental changes in camera pose, visual odometry enables systems to:
- Navigate in environments where GPS might be unavailable or unreliable.
- Build maps of unknown environments (in conjunction with SLAM techniques).
- Track an object's movement, which is vital in self-driving cars, drones, and mobile robots.

Visual odometry allows for real-time pose estimation using only camera input, making it a highly valuable technique in scenarios where lightweight and cost-effective sensors are needed.

### How Do We Calculate Visual Odometry?
The visual odometry algorithm typically involves the following steps:

1. **Capture Images**: The process begins with capturing consecutive frames at times \( t \) and \( t+1 \).
2. **Image Preprocessing**: Before processing, images are often undistorted based on the camera's intrinsic calibration data.
3. **Feature Detection**: Keypoints are detected in the first frame using algorithms like FAST. These keypoints are tracked in the subsequent frame.
4. **Essential Matrix Estimation**: Using the tracked keypoints and the 5-point algorithm, the essential matrix is computed with RANSAC for outlier rejection.
5. **Pose Recovery**: From the essential matrix, the rotational matrix and translation vector are estimated. These represent the camera's motion between frames.
6. **Scale Estimation**: Since monocular odometry can't recover scale directly, external data (e.g., from an accelerometer or speedometer) can be used to estimate the absolute scale of movement.

#### Monocular vs. Stereo Visual Odometry
- **Monocular Visual Odometry**: Uses a single camera, relies on image features for pose estimation, but cannot recover the absolute scale of movement.
- **Stereo Visual Odometry**: Uses two cameras to compute the pose through triangulation, allowing full recovery of the object’s motion, including scale.

For further reading on monocular visual odometry, see this [useful resource](https://avisingh599.github.io/vision/monocular-vo/).

## Installation
To run the monocular odometry binary, follow these steps:

1. Clone this repository to your local machine:
    ```
    git clone https://github.com/your-username/monocular_odometry_tutorial.git
    ```

2. Install the required dependencies. Make sure you have the following installed:
    - OpenCV (version 4.5.4)

3. Download the KITTI dataset from [here](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) and extract it to a directory of your choice.

4. Build the project by running the following command:
    ```
    make
    ```

**Note:** Before running the project, make sure to update the path in the main file (`monocular_odometry.cpp`) to match the location where you have installed the KITTI dataset.

## Usage
Once you have successfully installed the project, you can run the monocular odometry binary by executing the following command:
```
./monocular_odometry
```

Make sure you have a sequence of images in a specific format (e.g., JPEG, PNG) stored in the `data` directory. The binary will process these images and estimate the camera motion.

## Contributing
Contributions to this tutorial are welcome! If you find any issues or have suggestions for improvements, please feel free to open an issue or submit a pull request.

## License
This project is licensed under the [MIT License](LICENSE).

## Acknowledgements
This tutorial is inspired by the following repositories:
- [avisingh599/mono-vo](https://github.com/avisingh599/mono-vo/tree/master)
- [thehummingbird/robotics_demos](https://github.com/thehummingbird/robotics_demos/tree/main/monocular_vo)
