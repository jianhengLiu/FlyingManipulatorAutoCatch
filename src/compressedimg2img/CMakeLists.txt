cmake_minimum_required(VERSION 3.0.2)
project(compressedimg2img)

# # Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        cv_bridge
        nodelet
)

find_package(OpenCV REQUIRED)

catkin_package(
)

include_directories(

        # include
        ${catkin_INCLUDE_DIRS}
)

add_executable(compressedimg2img_node
        src/compressedimg2img_node.cpp
)
target_link_libraries(compressedimg2img_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_library(compressedimg2img_nodelet
        src/compressedimg2img_nodelet.cpp
)
target_link_libraries(compressedimg2img_nodelet ${catkin_LIBRARIES} ${OpenCV_LIBS})
