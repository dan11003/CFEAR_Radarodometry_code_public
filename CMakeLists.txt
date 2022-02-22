    cmake_minimum_required(VERSION 2.8.3)
project(cfear_radarodometry)
set (CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE RELEASE_OUTPUT_NAME)


if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
add_definitions(-std=c++14)
endif()
## If pcl not found
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")
find_package(PCL  REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    laser_geometry
    eigen_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    genmsg
    tf_conversions
    cv_bridge
    image_transport
    #ndt_map
    )
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -O3 ")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -O3")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
#set(DCMAKE_CXX_FLAGS "-Werror -Wall -Wextra")

find_package(Ceres REQUIRED)
find_package(PCL 1.0 REQUIRED)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

#find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
 
catkin_package(
    INCLUDE_DIRS include
    DEPENDS Eigen3 
    CATKIN_DEPENDS roscpp laser_geometry sensor_msgs tf_conversions tf geometry_msgs #ndt_map
    LIBRARIES ${PROJECT_NAME}
    )



include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

 set (${PROJECT_NAME}_LIB_SRCS
   src/${PROJECT_NAME}/utils.cpp
   src/${PROJECT_NAME}/pointnormal.cpp
   src/${PROJECT_NAME}/registration.cpp
   src/${PROJECT_NAME}/n_scan_normal.cpp
   src/${PROJECT_NAME}/radar_filters.cpp
   src/${PROJECT_NAME}/odometrykeyframefuser.cpp
   src/${PROJECT_NAME}/radar_driver.cpp
   src/${PROJECT_NAME}/eval_trajectory.cpp #
   src/${PROJECT_NAME}/statistics.cpp # used to measure execution time
 )

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}  ${CERES_LIBRARIES}) # ${Boost_LIBRARIES})

add_executable(eval_node src/eval_node.cpp)
target_link_libraries(eval_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${PROJECT_NAME})

add_executable(offline_odometry src/offline_odometry.cpp)
target_link_libraries(offline_odometry ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${PROJECT_NAME})

add_executable(cfear_radarodometry_node src/cfear_radarodometry_node.cpp)
target_link_libraries(cfear_radarodometry_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${PROJECT_NAME})

add_executable(radar_filter_node src/radar_filter_node.cpp)
target_link_libraries(radar_filter_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${PROJECT_NAME})#############

# Not tested but can be used to provide "Registration as a ros-service" 
#add_executable(reg_service_2d src/registration_srv_node.cpp)
#target_link_libraries(reg_service_2d ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${PROJECT_NAME})



