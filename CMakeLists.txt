cmake_minimum_required(VERSION 2.8.3)
project(p4_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  nav_msgs
  cmake_modules
  message_generation
)

add_message_files(DIRECTORY msg FILES
   PVA_request.msg
   PolyPVA.msg
   PVA.msg
   PVA_4d.msg
)

add_service_files( FILES
  minAccXYWpPVA.srv
  min_time.srv
  trapezoidal_p2p.srv
)

generate_messages( DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
)

find_package(osqp REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost ${BOOST_MIN_VERSION} COMPONENTS iostreams REQUIRED)

#add_subdirectory(P4)

catkin_package(
  DEPENDS Eigen message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  P4/src
  P4/examples
  TimeOptimizer/time_optimizer/src
  TimeOptimizer/time_optimizer/third_party/include
  TimeOptimizer/time_optimizer/third_party/include/ecos
  TimeOptimizer/time_optimizer/third_party/include/suitesparse
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories( ${PROJECT_SOURCE_DIR}/TimeOptimizer/time_optimizer/third_party/lib/mosek8_1 
                  ${PROJECT_SOURCE_DIR}/TimeOptimizer/time_optimizer/third_party/lib/ecos)


add_executable(p4_services src/p4_services_node.cpp
                           src/services.cpp
                           src/p4_helper.cpp
                           src/trapezoidal.cpp
                           src/time_optimizer_class.cpp
                           src/geometry_functions.cpp
                           src/visualization_functions.cpp
                           TimeOptimizer/time_optimizer/src/make_sparse.cpp
                           TimeOptimizer/time_optimizer/src/time_optimizer.cpp
                           TimeOptimizer/time_optimizer/src/time_optimizer_ecos.cpp
                           P4/src/polynomial_solver.cc
                           P4/src/polynomial_sampler.cc)
add_dependencies(p4_services ${catkin_EXPORTED_TARGETS})
add_dependencies(p4_services p4_ros_generate_messages_cpp)
target_link_libraries(p4_services ${catkin_LIBRARIES} ${Boost_LIBRARIES} 
                      osqp::osqp mosek64 libldl.a libecos.a)

add_executable(p4_min_time_client src/p4_min_time_client_example.cpp
                                  src/p4_helper.cpp
                                  src/trapezoidal.cpp
                                  src/geometry_functions.cpp
                                  TimeOptimizer/time_optimizer/src/time_optimizer.cpp
                                  P4/src/polynomial_solver.cc
                                  P4/src/polynomial_sampler.cc)
add_dependencies(p4_min_time_client ${catkin_EXPORTED_TARGETS})
add_dependencies(p4_min_time_client p4_ros_generate_messages_cpp)
target_link_libraries(p4_min_time_client ${catkin_LIBRARIES} ${Boost_LIBRARIES} osqp::osqp mosek64)

add_executable(p4_trapezoidal_client src/p4_trapezoidal_client_example.cpp)
add_dependencies(p4_trapezoidal_client ${catkin_EXPORTED_TARGETS})
add_dependencies(p4_trapezoidal_client p4_ros_generate_messages_cpp)
target_link_libraries(p4_trapezoidal_client ${catkin_LIBRARIES})