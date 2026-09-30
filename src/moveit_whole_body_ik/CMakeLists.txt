cmake_minimum_required(VERSION 3.0.2)
project(moveit_whole_body_ik)

add_compile_options(-std=c++14)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  kdl_parser
  message_generation
  moveit_core
  moveit_msgs
  moveit_ros_planning
  octomap_msgs
  pcl_conversions
  roscpp
  sensor_msgs
  tf2_ros
  visualization_msgs
)

add_service_files(
  FILES
  SolveMoveItIK.srv
  SolveSampledWholeBodyIK.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs kdl_parser message_runtime moveit_core moveit_msgs
    moveit_ros_planning octomap_msgs pcl_conversions roscpp
    sensor_msgs tf2_ros visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(moveit_ik_service_node src/moveit_ik_service_node.cpp)
add_dependencies(moveit_ik_service_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(moveit_ik_service_node ${catkin_LIBRARIES})

add_executable(sampled_whole_body_ik_service_node
  src/sampled_whole_body_ik_service_node.cpp)
add_dependencies(sampled_whole_body_ik_service_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sampled_whole_body_ik_service_node ${catkin_LIBRARIES})

add_executable(pcd_octomap_planning_scene_node
  src/pcd_octomap_planning_scene_node.cpp)
add_dependencies(pcd_octomap_planning_scene_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcd_octomap_planning_scene_node
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} octomap)

install(TARGETS
  moveit_ik_service_node
  sampled_whole_body_ik_service_node
  pcd_octomap_planning_scene_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
