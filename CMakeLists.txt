cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sim_visualization)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  foxglove_msgs
  gazebo_msgs
  geometry_msgs
  mavros_msgs
  roscpp
  roslaunch
  std_msgs
  tf2_ros
  visualization_msgs
  xgc2_robot_visualization
)

catkin_package(
  CATKIN_DEPENDS foxglove_msgs gazebo_msgs geometry_msgs mavros_msgs roscpp std_msgs tf2_ros visualization_msgs xgc2_robot_visualization
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gazebo_auto_visualizer_node
  src/gazebo_auto_visualizer_node.cpp
)
add_dependencies(gazebo_auto_visualizer_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gazebo_auto_visualizer_node
  ${catkin_LIBRARIES}
)

install(TARGETS gazebo_auto_visualizer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch/gazebo_auto_visualization_rviz.launch)
endif()
