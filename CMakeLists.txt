cmake_minimum_required(VERSION 3.10)
project(oak_ros)

# enable C++17 standard (current directory scope)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# https://stackoverflow.com/questions/5395309/how-do-i-force-cmake-to-include-pthread-option-during-compilation
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG ON)

# configure build types and build c-flags
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE
      Debug
      CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
                                               "MinSizeRel" "RelWithDebInfo")
endif()

find_package(Threads REQUIRED)
find_package(OpenCV 4 REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport
                                        cv_bridge pcl_conversions pcl_ros tf2 tf2_ros)

# running dependency
catkin_package(# INCLUDE_DIRS LIBRARIES CATKIN_DEPENDS imu_vn_100 DEPENDS
)

# Finding depthai dependencies
set(depthai_DIR "/home/$ENV{USER}/git/depthai-core-multicam/build/install/lib/cmake/depthai")

find_package(depthai CONFIG REQUIRED)
if(depthai_FOUND)
  message(WARNING "depthai FOUND, Version:" ${depthai_VERSION} " directory: "
                  ${depthai_DIR})
endif()

# find_package(OpenCV REQUIRED)

# find_package(spdlog REQUIRED)

# Build Executables Below

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(lib_oak_ros src/oak_ros/OakRos.cpp src/oak_ros/OakRosFactory.cpp)
target_link_libraries(lib_oak_ros PUBLIC depthai::opencv ${catkin_LIBRARIES} Threads::Threads)

# alias will make public dependencies go through
add_library(lib_oak_ros::lib_oak_ros ALIAS lib_oak_ros)

add_executable(oak_ros src/main.cpp)

# linking depthai::core for proper header includes, and spdlog dependencies
target_link_libraries(oak_ros 
PRIVATE
lib_oak_ros::lib_oak_ros Threads::Threads
)

add_executable(test_stereo_depth_video test/test_stereo_depth_video.cpp)
target_link_libraries(test_stereo_depth_video 
PRIVATE
lib_oak_ros::lib_oak_ros
Threads::Threads
)

add_executable(test_calibration_reader test/test_calibration_reader.cpp)
target_link_libraries(test_calibration_reader 
PRIVATE
lib_oak_ros::lib_oak_ros
Threads::Threads
)