cmake_minimum_required(VERSION 2.8.3)
project(i_see_pee)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -march=native)

set(I_SEE_PEE_DEPS nav_msgs tf2 tf2_ros tf2_geometry_msgs roscpp geometry_msgs sensor_msgs)
find_package(catkin REQUIRED ${I_SEE_PEE_DEPS})
find_package(Eigen3 REQUIRED)


###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${I_SEE_PEE_DEPS}
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
)

add_library(${PROJECT_NAME} src/i_see_pee.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/i_see_pee_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

#############
## Testing ##
#############

catkin_add_gtest(${PROJECT_NAME}-test
    test/icp_converion_checker.cpp
    test/icp_point_to_map.cpp
    test/internal_stamp.cpp
    test/iterator.cpp
    test/map_converter.cpp
    test/map_interpret.cpp
    test/odom_interface_tf.cpp
    test/scan_interface.cpp
    test/utils.cpp
    )
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${GTEST_MAIN_LIBRARIES})

add_executable(odom_publisher_tf test/integration/odom_publisher_tf.cpp)
target_link_libraries(odom_publisher_tf ${PROJECT_NAME})

add_executable(knn test/integration/knn.cpp)
target_link_libraries(knn ${PROJECT_NAME})