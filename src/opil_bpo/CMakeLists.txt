cmake_minimum_required(VERSION 2.8.3)
project(opil_bpo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  nav_msgs
  )

add_message_files(
   FILES
   AoI.msg
   PathDist.msg
   Results.msg
)

add_service_files(
   FILES
   shpath.srv
   tppath.srv 
   taskplanner_req.srv)

generate_messages(
   DEPENDENCIES
   std_msgs
   nav_msgs
   sensor_msgs
   geometry_msgs
 )


catkin_package(
   CATKIN_DEPENDS 
   message_runtime
   roscpp
   std_msgs
   sensor_msgs
   rospy
)


include_directories(/opt/ros/kinetic/lib)
include_directories(/opt/ros/kinetic/include)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(service_path_finder2 src/pathdist_v2.cpp)
add_dependencies(service_path_finder2 opil_bpo_generate_messages_cpp opil_bpo_gencpp)
add_dependencies(service_path_finder2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(service_path_finder2 ${catkin_LIBRARIES})

add_executable(annotfilt src/parser_v1.cpp)
add_dependencies(annotfilt opil_bpo_generate_messages_cpp opil_bpo_gencpp)
target_link_libraries(annotfilt ${catkin_LIBRARIES})
