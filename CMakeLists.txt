cmake_minimum_required(VERSION 2.8.3)
project(contact_point_estimation)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs orocos_kdl std_srvs eigen_utils)

find_package(Eigen REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  DEPENDS eigen
  CATKIN_DEPENDS roscpp geometry_msgs orocos_kdl std_srvs eigen_utils
  INCLUDE_DIRS include
  LIBRARIES contact_point_estimation surface_tracing_controller
)

add_executable(contact_point_estimation_node src/ContactPointEstimationParams.cpp src/ContactPointEstimation.cpp src/contact_point_estimation_node.cpp)
target_link_libraries(contact_point_estimation_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(contact_point_estimation src/ContactPointEstimationParams.cpp src/ContactPointEstimation.cpp)
target_link_libraries(contact_point_estimation ${catkin_LIBRARIES})

add_library(surface_tracing_controller src/SurfaceTracingController.cpp)
target_link_libraries(surface_tracing_controller ${catkin_LIBRARIES})

install(TARGETS contact_point_estimation_node surface_tracing_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)