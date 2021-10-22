cmake_minimum_required(VERSION 3.0.2)
project(map_topic_processing)


find_package(catkin REQUIRED COMPONENTS
  roscpp
	std_msgs
	cv_bridge
	image_transport
)

find_package(OpenCV REQUIRED)

#set(OpenCV_DIR /

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_topic_processing
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
	INCLUDE_DIRS include
	LIBRARIES opencv
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)


add_executable(map_processing src/map_processing.cpp)
target_link_libraries(map_processing ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(image_processing src/image_processing.cpp)
target_link_libraries(image_processing ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
