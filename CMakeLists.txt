cmake_minimum_required(VERSION 2.8.3)
project(KCF)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   genmsg
   message_generation
   sensor_msgs
   cv_bridge
   image_transport  
)
find_package(OpenCV REQUIRED)

## Declare ROS messages and services
add_message_files (DIRECTORY msg FILES ActuatorControl.msg BoundingBox.msg )
## Generate added messages and services

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
) 

## Declare a catkin package

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES KCF
  DEPENDS system_lib opencv
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(KCF_UAV_node src/runtracker.cpp src/kcftracker.cpp src/fhog.cpp)
target_link_libraries(KCF_UAV_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(KCF_UAV_node KCF_generate_messages_cpp )


add_executable(KCF_ground_node src/ground.cpp src/kcftracker.cpp src/fhog.cpp)
target_link_libraries(KCF_ground_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(KCF_ground_node KCF_generate_messages_cpp )



  








