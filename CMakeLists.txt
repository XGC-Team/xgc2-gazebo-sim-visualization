cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sim_visualization)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  foxglove_msgs
  gazebo_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  xgc2_robot_visualization
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_scene_contract
  CATKIN_DEPENDS foxglove_msgs gazebo_msgs geometry_msgs mavros_msgs nav_msgs roscpp std_msgs tf2_geometry_msgs tf2_ros visualization_msgs xgc2_robot_visualization
)

# xgc2_robot_visualization is checked out beside this package and also exists in
# the installed ROS overlay. catkin_INCLUDE_DIRS puts the overlay prefix ahead of
# workspace packages, so without naming the dependency first a development build
# compiles against the installed headers -- a field added in the checkout is
# invisible to its own consumer in the same build, and the failure reads as a
# code error rather than a search order. Naming it is correct in an installed
# build too, where it simply resolves to the same prefix.
include_directories(
  include
  ${xgc2_robot_visualization_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(gazebo_scene_contract
  src/scene_contract.cpp
)
add_dependencies(gazebo_scene_contract
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gazebo_scene_contract
  ${catkin_LIBRARIES}
)

add_executable(gazebo_auto_visualizer_node
  src/gazebo_auto_visualizer_node.cpp
)
add_dependencies(gazebo_auto_visualizer_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gazebo_auto_visualizer_node
  gazebo_scene_contract
  ${catkin_LIBRARIES}
)

install(TARGETS gazebo_scene_contract gazebo_auto_visualizer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  # roslaunch_add_file_check embeds argument text in its CMake target name, where
  # '#' is illegal. This value only satisfies required-argument XML validation;
  # the node's strict canonical #rrggbb parser is covered by scene_contract_test.
  roslaunch_add_file_check(launch/gazebo_auto_visualization_rviz.launch "marker_color:=000000")
  catkin_add_gtest(scene_contract_test test/scene_contract_test.cpp)
  if(TARGET scene_contract_test)
    target_link_libraries(scene_contract_test gazebo_scene_contract ${catkin_LIBRARIES})
  endif()
endif()
