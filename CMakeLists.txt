cmake_minimum_required(VERSION 2.8.3)
project(cbf_cassie_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  # message(STATUS "=============================================Eigen path: ${EIGEN3_INCLUDE_DIR}")
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  # message(STATUS "=============================================Eigen path: ${EIGEN3_INCLUDE_DIR}")
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    visualization_msgs
    roslib
    grid_map_ros
    grid_map_core
    grid_map_msgs
    sensor_msgs
    inekf_msgs
    planner_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   DEPENDS
   grid_map_ros
   grid_map_core
   grid_map_msgs
   Eigen3
   std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
include_directories(/usr/local/lib)
include_directories(/usr/local/include/)

####################################################################################
#### TEST QP
####################################################################################
add_executable(cbf_test_qp src/test_QP.cpp src/cbf.cpp src/QuadProg++.cc src/Array.cc
)
add_dependencies(cbf_test_qp ${${PROJECT_NAME}robot_pose_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})
target_link_libraries(cbf_test_qp ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

####################################################################################
#### TEST Pipeline
####################################################################################
add_executable(cbf_test_pipeline src/main.cpp src/driver.cpp src/map_utils.cpp
src/communication.cpp src/planner_info_to_controller_t.c src/utils/plotting.cpp
src/cbf.cpp src/QuadProg++.cc src/Array.cc
)
add_dependencies(cbf_test_pipeline ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS})
target_link_libraries(cbf_test_pipeline ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

####################################################################################
#### Fake Robot Publisher
####################################################################################
add_executable(cbf_fake_robot_publisher src/fake_robot_publisher.cpp src/fake_robot.cpp
)
add_dependencies(cbf_fake_robot_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})
target_link_libraries(cbf_fake_robot_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

####################################################################################
#### Fake Map Publisher
####################################################################################
add_executable(cbf_fake_map_publisher src/fake_map_publisher.cpp src/fake_map.cpp
)
add_dependencies(cbf_fake_map_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})
target_link_libraries(cbf_fake_map_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

####################################################################################
#### TEST Simulation
####################################################################################
add_executable(cbf_test_simulation src/main.cpp src/simutest.cpp src/map_utils.cpp
src/communication.cpp src/planner_info_to_controller_t.c src/utils/plotting.cpp
src/cbf.cpp src/QuadProg++.cc src/Array.cc
)
add_dependencies(cbf_test_simulation ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})
target_link_libraries(cbf_test_simulation ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)
