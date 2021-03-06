cmake_minimum_required(VERSION 2.8.3)
project(ur5_move)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  genmsg
  moveit_core
  moveit_ros_move_group
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  std_msgs
  tf
  ur_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(RSB 0.11 REQUIRED)
find_package(RSC 0.11 REQUIRED)
find_package(RST 0.11 REQUIRED)

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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


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
#  INCLUDE_DIRS include
#  LIBRARIES ur5_move
#  CATKIN_DEPENDS genmsg moveit_core moveit_ros_move_group moveit_ros_planning moveit_ros_planning_interface roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(BEFORE SYSTEM
${RSB_INCLUDE_DIRS} ${RST_INCLUDE_DIRS}
)
ADD_DEFINITIONS(${RST_CFLAGS})

## Declare a cpp library
# add_library(ur5_move
#   src/${PROJECT_NAME}/ur5_move.cpp
# )


# mark-start::define-target-names
SET(LIBRARY_NAME "ur5-microservice")
SET(BINARY_NAME_SERVICE "rsb-ur5-microservice")
SET(BINARY_NAME_CLIENT "rsb-ur5-microservice-client")
SET(EXPORT_NAME "ur5-microservice-exports")

# mark-start::declare-library
SET(LIBRARY_SOURCES "src/ur5-microservice/library/ur5-rsb-adapter.cpp")
SET(LIBRARY_HEADERS "src/ur5-microservice/library/ur5-rsb-adapter.h")
ADD_LIBRARY(${LIBRARY_NAME} SHARED ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
#SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES VERSION ${SO_VERSION})
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${RSC_LIBRARIES} ${RSB_LIBRARIES} ${RST_LIBRARIES} ${catkin_LIBRARIES})
# mark-end::declare-library

# mark-start::declare-executable
ADD_EXECUTABLE(${BINARY_NAME_SERVICE} "src/ur5-microservice/ur5-microservice.cpp")
TARGET_LINK_LIBRARIES(${BINARY_NAME_SERVICE} ${LIBRARY_NAME} ${RSB_LIBRARIES} ${RST_LIBRARIES})
# mark-end::declare-executable

# mark-start::declare-executable
ADD_EXECUTABLE(${BINARY_NAME_CLIENT} "src/ur5-microservice/ur5-microservice-client.cpp")
TARGET_LINK_LIBRARIES(${BINARY_NAME_CLIENT} ${LIBRARY_NAME} ${RSB_LIBRARIES} ${RST_LIBRARIES})
# mark-end::declare-executable

## Declare a cpp executable
 add_executable(ur5_pick src/ur5_pick.cpp)
 add_executable(ur5_pick_out src/ur5_pick_out.cpp)
 add_executable(ur5_init src/ur5_init.cpp)
 add_executable(ur5_place src/ur5_place.cpp)
 add_executable(ur5_place_out src/ur5_place_out.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ur5_move_node ur5_move_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(ur5_pick
   ${catkin_LIBRARIES}
 )

 target_link_libraries(ur5_pick_out
   ${catkin_LIBRARIES}
 )

 target_link_libraries(ur5_init
   ${catkin_LIBRARIES}
 )

 target_link_libraries(ur5_place
   ${catkin_LIBRARIES}
 )

 target_link_libraries(ur5_place_out
   ${catkin_LIBRARIES}
 )

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
# install(TARGETS ur5_move ur5_move_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ur5_move.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
