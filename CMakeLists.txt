cmake_minimum_required(VERSION 2.8.3)
project(coverage_2d)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  costmap_2d
  map_server
  roscpp
  rospy
  tf
  nav_msgs
  nav2d_navigator
  nav2d_exploration
  nav2d_karto
  nav2d_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Point2D.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ComputeCentroids.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  nav_msgs
  nav2d_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES coverage_2d
  CATKIN_DEPENDS message_runtime costmap_2d map_server roscpp rospy tf nav_msgs nav2d_msgs nav2d_navigator nav2d_exploration nav2d_karto
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(coverage_2dlib
  src/${PROJECT_NAME}/CoverageServer.cpp
  src/${PROJECT_NAME}/geometry.cpp
)
add_dependencies(coverage_2dlib coverage_2dlib_gencpp)
target_link_libraries(coverage_2dlib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Declare a cpp executable
add_executable(coverage_2d src/coverage_2d.cpp)
add_dependencies(coverage_2d coverage_2d)
target_link_libraries(coverage_2d coverage_2dlib ${catkin_LIBRARIES})

add_executable(coverage_server src/coverage_server.cpp)
add_dependencies(coverage_server coverage_2d)
target_link_libraries(coverage_server coverage_2dlib ${catkin_LIBRARIES})

add_executable(single_explorer src/coverage_2d/test/single_explorer.cpp)
add_dependencies(single_explorer coverage_2d)
target_link_libraries(single_explorer coverage_2dlib ${catkin_LIBRARIES})

add_executable(pair_exploration src/coverage_2d/test/pair_exploration.cpp)
add_dependencies(pair_exploration coverage_2d)
target_link_libraries(pair_exploration coverage_2dlib ${catkin_LIBRARIES})

add_executable(map_validation src/coverage_2d/test/map_validation.cpp)
add_dependencies(map_validation coverage_2d)
target_link_libraries(map_validation coverage_2dlib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Add all of the demos
add_executable(coverage_demo src/coverage_2d/demo/coverage_demo.cpp)
add_dependencies(coverage_demo coverage_2d)
target_link_libraries(coverage_demo coverage_2dlib ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(coverage_2d_node coverage_2d_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(coverage_2d
#   ${catkin_LIBRARIES}
# )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS coverage_2d coverage_2d_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_coverage_2d.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
