cmake_minimum_required(VERSION 3.0.2)
project(custom_costmap_layer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  pluginlib
  dynamic_reconfigure
  visualization_msgs
)

# Generate dynamic reconfigure options
generate_dynamic_reconfigure_options(
  cfg/CustomLayer.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES custom_costmap_layer
  CATKIN_DEPENDS roscpp costmap_2d pluginlib dynamic_reconfigure visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/devel/include # Add this line
)

add_library(${PROJECT_NAME} src/custom_layer.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Dynamic reconfigure: make sure configure headers are built before any node using them
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
