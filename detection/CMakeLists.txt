cmake_minimum_required(VERSION 2.8)

project( detection )

find_package( OpenCV REQUIRED )
find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    rospy
    std_msgs
    message_filters
    pluginlib
    rosconsole
    roslib
    sensor_msgs
    image_transport
    tf
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable( detection  main.cpp )

target_link_libraries(detection ${OpenCV_LIBS} ${catkin_LIBRARIES})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
