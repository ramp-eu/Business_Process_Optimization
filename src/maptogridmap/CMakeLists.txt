cmake_minimum_required(VERSION 2.8.3)
project(maptogridmap)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  nav_msgs
#  maptogridmap
  )

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Vertex.msg
   Edge.msg
   Graph.msg
   Nodes.msg
   Edges.msg
)

## Generate services in the 'srv' folder
#add_service_files(
#   FILES
#   shpath.srv
#   tppath.srv 
#   taskplanner_req.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   nav_msgs
   sensor_msgs
   geometry_msgs
#   maptogridmap
 )


catkin_package(
   CATKIN_DEPENDS 
   message_runtime
   roscpp
   std_msgs
   sensor_msgs
   rospy
)


#include_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
#include_directories(include/${PROJECT_NAME})
#include_directories(${PROJECT_NAME}/include)
include_directories(/opt/ros/kinetic/lib)
include_directories(/opt/ros/kinetic/include)
include_directories(include ${catkin_INCLUDE_DIRS})

#executable ROS Service
#add_executable(service_path_finder2 src/pathdist_v2.cpp)
#add_dependencies(service_path_finder2 opil_bpo_generate_messages_cpp opil_bpo_gencpp) #maptogridmap_gencpp)
#add_dependencies(service_path_finder2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(service_path_finder2 ${catkin_LIBRARIES})

#executable ROS Client
#add_executable(path_client src/pathdist_v01.cpp)
#add_dependencies(client_path_finder bpo_gencpp maptogridmap_gencpp)
#add_dependencies(client_path_finder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(client_path_finder ${catkin_LIBRARIES})

## Declare a cpp executable
#add_executable(annotfilt src/parser_v1.cpp)
#add_dependencies(annotfilt opil_bpo_generate_messages_cpp opil_bpo_gencpp)
#target_link_libraries(annotfilt ${catkin_LIBRARIES})


