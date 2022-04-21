cmake_minimum_required(VERSION 3.0.2)
project(IPM-mapping-ros)

# 1.find package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosmsg
  rospy
  message_filters # message_filters
  cv_bridge # ros cv_bridge
  OpenCV # ros cv
  image_transport # ros cv
)

# PCL
find_package(PCL REQUIRED COMPONENTS 
  common 
  io
  visualization
) 
# OPENCV
find_package(OpenCV REQUIRED) 
message(STATUS "version: ${OpenCV_VERSION}")

find_package(Boost REQUIRED)

# 2.catkin package
catkin_package()

# 3.include directories
include_directories(
  include

  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# add_executable(image_sub  src/image_sub.cpp)
# target_link_libraries(image_sub  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(image2point src/image2point.cpp)
target_link_libraries(image2point ${OpenCV_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(pcl_view src/pcl_view.cpp)
# target_link_libraries(pcl_view ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES})


