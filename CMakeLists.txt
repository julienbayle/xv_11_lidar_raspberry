cmake_minimum_required(VERSION 2.8.3)
project(xv_11_lidar_raspberry)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
  LIBRARIES
    Lidar
    LidarMotor
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

# Motor
add_library(LidarMotor src/motor.cpp)

# Lidar
add_library(Lidar src/lidar.cpp)
target_link_libraries(Lidar
  LidarMotor
  ${catkin_LIBRARIES}
)

# Lidar node
add_executable(${PROJECT_NAME} src/lidar_node.cpp)
target_link_libraries(${PROJECT_NAME}
  Lidar
  LidarMotor
  ${catkin_LIBRARIES}
)

# Install includes
install(DIRECTORY include/${PROJECT_NAME} 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install libraries
install(TARGETS Lidar LidarMotor
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executable
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
  launch
  cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)