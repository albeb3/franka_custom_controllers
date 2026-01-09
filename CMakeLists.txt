cmake_minimum_required(VERSION 3.4)
project(franka_custom_controllers)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  joint_limits_interface
  tf
  tf_conversions
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
  urdf
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

# merge Franka + INCLUDE_DIRS in topological order
list_insert_in_workspace_order(catkin_INCLUDE_DIRS ${Franka_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_custom_controllers
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    eigen_conversions
    franka_hw
    franka_gripper
    geometry_msgs
    hardware_interface
    joint_limits_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    urdf
    visualization_msgs
  DEPENDS Franka
)

add_library(franka_custom_controllers
  src/joint_velocity_controller.cpp
)

add_dependencies(franka_custom_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_custom_controllers PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(franka_custom_controllers SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(franka_custom_controllers PUBLIC
  include
)



add_executable(tf_broadcaster_node src/tf_broadcaster_node.cpp)
add_executable(tf_EE_node src/tf_EE_node.cpp)
add_executable(tf_EE_velocity_node src/tf_EE_velocity_node.cpp)
add_executable(tf_EE_velocity_node_Left src/tf_EE_velocity_node_Left.cpp)
add_executable(tf_EE_velocity_node_Right src/tf_EE_velocity_node_Right.cpp)
add_executable(tf_links_wrt_world_node src/tf_links_wrt_world.cpp)
add_executable(gripper_node src/gripper_node.cpp)


target_link_libraries(tf_broadcaster_node
  ${catkin_LIBRARIES}
)
target_link_libraries(tf_EE_node
  ${catkin_LIBRARIES}
)

target_link_libraries(tf_EE_velocity_node
  ${catkin_LIBRARIES}
)
target_link_libraries(tf_EE_velocity_node_Left
  ${catkin_LIBRARIES}
)
target_link_libraries(tf_EE_velocity_node_Right
  ${catkin_LIBRARIES}
)

target_link_libraries(tf_links_wrt_world_node
  ${catkin_LIBRARIES}
)
target_link_libraries(gripper_node
  ${catkin_LIBRARIES}
)
## Installation
install(TARGETS franka_custom_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES franka_custom_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(
  PROGRAMS scripts/interactive_joystick.py  scripts/Plot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(franka_custom_controllers FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_custom_controllers
    FILES ${SOURCES}
    DEPENDS franka_custom_controllers
  )
endif()

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/PepTools.cmake OPTIONAL
  RESULT_VARIABLE PEP_TOOLS
)
if(PEP_TOOLS)
  file(GLOB_RECURSE PYSOURCES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
  add_pyformat_target(franka_control FILES ${PYSOURCES})
endif()
