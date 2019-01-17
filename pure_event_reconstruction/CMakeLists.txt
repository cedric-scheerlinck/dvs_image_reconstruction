cmake_minimum_required(VERSION 2.8.3)
project(pure_event_reconstruction)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)

# make libraries
cs_add_library(${PROJECT_NAME} src/pure_event_reconstruction.cpp
							   src/bag_player.cpp
							   src/utils.cpp)

# link required libraries
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

# make the executable
cs_add_executable(reconstruction_node src/reconstruction_node.cpp)

# link required libraries to the executable
target_link_libraries(reconstruction_node ${PROJECT_NAME})

cs_install()
