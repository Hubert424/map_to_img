cmake_minimum_required(VERSION 2.8.3)
project(map_to_img)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS cv_bridge geometry_msgs hector_map_tools image_transport nav_msgs sensor_msgs)
find_package(Eigen3)
find_package(OpenCV REQUIRED)

catkin_package(
)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(map_to_img src/map_to_img.cpp)

target_link_libraries(map_to_img
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(dummy_pose_publisher src/dummy_pose_publisher.cpp)

target_link_libraries(dummy_pose_publisher
  ${catkin_LIBRARIES}
)
