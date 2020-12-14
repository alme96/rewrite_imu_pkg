cmake_minimum_required(VERSION 3.0.2)
project(rewrite_imu_pkg)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs nav_msgs)

find_package(Boost REQUIRED)
find_package(Armadillo REQUIRED)

catkin_package()

## add_library( evaluation
##   src/evaluation.cpp
## )

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS})

##add_executable(evaluation_node src/evaluation_node.cpp)
##target_link_libraries(evaluation_node evaluation ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})

add_executable(rewrite_imu_node src/rewrite_imu_node.cpp)
target_link_libraries(rewrite_imu_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
