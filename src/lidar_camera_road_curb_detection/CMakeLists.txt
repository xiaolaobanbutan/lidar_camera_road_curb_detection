cmake_minimum_required(VERSION 3.0.2)
project(lidar_camera_road_curb_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy sensor_msgs geometry_msgs std_msgs visualization_msgs
  pcl_conversions pcl_ros cv_bridge image_transport message_filters
  laser_geometry tf jsk_recognition_msgs
)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui dnn)
find_package(Armadillo REQUIRED)
find_package(OpenMP)
find_path(MLPACK_INCLUDE_DIR mlpack.hpp)
find_library(MLPACK_LIBRARY mlpack)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs std_msgs visualization_msgs
                 pcl_conversions pcl_ros cv_bridge image_transport message_filters
                 laser_geometry tf jsk_recognition_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

file(GLOB NODE_SOURCES CONFIGURE_DEPENDS src/*.cpp)
if(NOT MLPACK_INCLUDE_DIR OR NOT MLPACK_LIBRARY)
  list(REMOVE_ITEM NODE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/bayes_regression.cpp")
  message(STATUS "mlpack not found: Bayesian refinement disabled; RANSAC curb points retained")
endif()
add_executable(${PROJECT_NAME} ${NODE_SOURCES})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

if(MLPACK_INCLUDE_DIR AND MLPACK_LIBRARY)
  target_include_directories(${PROJECT_NAME} PRIVATE ${MLPACK_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} ${MLPACK_LIBRARY})
  target_compile_definitions(${PROJECT_NAME} PRIVATE ROAD_CURB_HAVE_MLPACK=1)
endif()
