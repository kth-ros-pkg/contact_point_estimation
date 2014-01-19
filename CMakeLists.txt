cmake_minimum_required(VERSION 2.8.3)
project(contact_point_estimation)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs tf tf_conversions eigen_conversions tf2_geometry_msgs tf2_kdl std_msgs adaptive_control_utils)

find_package(Eigen REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  DEPENDS eigen
  CATKIN_DEPENDS roscpp geometry_msgs tf tf_conversions eigen_conversions tf2_geometry_msgs tf2_kdl std_msgs adaptive_control_utils
  INCLUDE_DIRS include
  LIBRARIES
)

add_executable(contact_point_estimation_node src/ContactPointEstimationParams.cpp src/ContactPointEstimation.cpp src/contact_point_estimation_node.cpp)
target_link_libraries(contact_point_estimation_node ${catkin_LIBRARIES})

install(TARGETS contact_point_estimation_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)