cmake_minimum_required(VERSION 2.8)
project( inference_threaded )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( inference_threaded  inference_threaded.cpp )
target_link_libraries( inference_threaded ${OpenCV_LIBS} )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
