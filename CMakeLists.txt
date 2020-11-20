cmake_minimum_required(VERSION 3.0.2)
project(rewrite_imu_pkg)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

find_package(Boost REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

##add_executable(imu_listener_node src/imu_listener_node.cpp)
##target_link_libraries(imu_listener_node ${catkin_LIBRARIES})

add_executable(rewrite_imu_node src/rewrite_imu_node.cpp)
target_link_libraries(rewrite_imu_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
