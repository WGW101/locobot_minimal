cmake_minimum_required(VERSION 3.0.2)
project(locobot_hardware_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  dynamixel_sdk
  hardware_interface
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dynamixel_sdk hardware_interface roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/locobot_controller_manager.cpp src/locobot_hardware_interface.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES
  OUTPUT_NAME locobot_controller_manager
  PREFIX ""
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
