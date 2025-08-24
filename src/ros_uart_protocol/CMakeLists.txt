cmake_minimum_required(VERSION 3.0.2)
project(ros_uart_protocol)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  serial
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf serial
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/uart_protocol_ros.cpp
  src/uart_ros_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)