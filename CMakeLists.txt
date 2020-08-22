cmake_minimum_required(VERSION 3.2)
project(li_calib)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -msse4.2")
set(CMAKE_BUILD_TYPE "RELEASE")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -msse4.2 -mavx")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rosbag
  geometry_msgs
  nav_msgs
  velodyne_msgs
  ndt_omp
  tf
  pcl_ros
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

set(PANGOLIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/build-pangolin")
find_package(Pangolin REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Kontiki)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${Pangolin_LIBRARIES}
        Kontiki
)

catkin_package(
   INCLUDE_DIRS include
#   LIBRARIES li_calibr_lib
#   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs velodyne_msgs
#  DEPENDS system_lib
)

include_directories( include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
)

add_library(li_calib_lib
        src/core/trajectory_manager.cpp
        src/core/inertial_initializer.cpp
        src/core/lidar_odometry.cpp
        src/core/surfel_association.cpp
)
target_link_libraries(li_calib_lib ${thirdparty_libraries})

add_executable(li_calib_gui
        test/li_calib_gui.cpp
        src/ui/calib_helper.cpp
        src/ui/calib_ui.cpp)
target_link_libraries(li_calib_gui li_calib_lib ${thirdparty_libraries})