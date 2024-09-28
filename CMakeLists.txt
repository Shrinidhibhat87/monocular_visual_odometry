cmake_minimum_required(VERSION 3.0)  # Specifies the minimum required version of CMake.
project(MonocularVisualOdometry)  # Sets the project name.

find_package(OpenCV REQUIRED)  # Finds the OpenCV package and sets the required variables.

include_directories(include ${OpenCV_INCLUDE_DIRS})  # Includes the specified directories for header files.

add_library(${PROJECT_NAME} SHARED source/visual_odometry.cpp)  # Adds a shared library target named ${PROJECT_NAME} with the source file visual_odometry.cpp.

add_executable(${PROJECT_NAME}_node main.cpp)  # Adds an executable target named ${PROJECT_NAME}_node with the source file main.cpp.

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${OpenCV_LIBS})  # Links the specified libraries to the ${PROJECT_NAME}_node target.
