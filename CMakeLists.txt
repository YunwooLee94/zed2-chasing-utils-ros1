cmake_minimum_required(VERSION 3.0.2)
project(zed_chasing_utils)

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++14")
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(Eigen3  REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


if(OpenCV_VERSION VERSION_LESS "4.0")
    add_definitions(-DOPENCV_VERSION_3_ZED)
else()
    add_definitions(-DOPENCV_VERSION_4_ZED)
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  roscpp
  tf
  tf_conversions
  visualization_msgs
        message_filters
        image_transport
        image_geometry
        sensor_msgs
        zed_interfaces
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
        LIBRARIES
        INCLUDE_DIRS include
        CATKIN_DEPENDS
)

file(GLOB ZED_CLIENT src/zed_chasing_utils/*.cpp)

add_executable(zed_client src/exe/main.cpp ${ZED_CLIENT} include/zed_chasing_utils/ChasingClient.h src/zed_chasing_utils/ChasingClient.cpp include/utils/math_util.h src/utils/math_util.cpp)
target_link_libraries(zed_client ${catkin_LIBRARIES})

add_executable(zed_tf_bridge src/exe/tf_bridge_main.cpp)
target_link_libraries(zed_tf_bridge ${catkin_LIBRARIES})