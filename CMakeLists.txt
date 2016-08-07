cmake_minimum_required(VERSION 2.8.3)
project(anpl_inf)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  sensor_msgs
  nav_msgs
  message_generation
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(BOOST_LIBRARIES boost_system boost_serialization)
set(GTSAM_LIBRARIES gtsam ${BOOST_LIBRARIES} tbb tbbmalloc)

find_package(PCL 1.7 REQUIRED)

if(${ARCHITECTURE} MATCHES ${odroid} )
	set(OMPL_LIBRARIES ompl ${BOOST_LIBRARIES} "-Wl,-rpath $ENV{HOME}/prefix/lib/arm-linux-gnueabihf")
else()
	set(OMPL_LIBRARIES_OLD ompl ${BOOST_LIBRARIES} "-Wl,-rpath $ENV{HOME}/prefix/lib/x86_64-linux-gnu")
endif()

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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
 add_service_files(
   FILES
   TurtlebotControllerService.srv
#   Service1.srv
#   Service2.srv
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
   sensor_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
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
#  LIBRARIES anpl_inf
#  CATKIN_DEPENDS roscpp rospy std_msg
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

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(~/prefix/include)
include_directories(src/test_icp/include)

link_directories(~/prefix/lib)

if(${ARCHITECTURE} MATCHES ${odroid} )
	set(GTSAM_COMPILE_FLAGS "-Wa,-mimplicit-it=thumb")
	add_definitions(${GTSAM_COMPILE_FLAGS})
	link_directories(~/prefix/lib/arm-linux-gnueabihf)	
else()
	link_directories(~/prefix/lib/x86_64-linux-gnu)
endif()

find_package(ompl)

add_executable(tal src/tal.cpp)
target_link_libraries(tal ${catkin_LIBRARIES})

add_executable(OdometryExample src/OdometryExample.cpp)
target_link_libraries(OdometryExample ${GTSAM_LIBRARIES})

add_executable(LocalizationExample src/LocalizationExample.cpp)
target_link_libraries(LocalizationExample ${GTSAM_LIBRARIES})

add_executable(easyPoint2KalmanFilter src/easyPoint2KalmanFilter.cpp)
target_link_libraries(easyPoint2KalmanFilter ${GTSAM_LIBRARIES})

add_executable(match_pcl src/match_pcl.cpp)
target_link_libraries(match_pcl ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(incrementally_pairs_of_clouds src/incrementally_pairs_of_clouds.cpp)
target_link_libraries(incrementally_pairs_of_clouds ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(icp_pcl_check src/icp_pcl_check.cpp)
target_link_libraries(icp_pcl_check ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_icp src/test_icp/test_program.cpp)
target_link_libraries(test_icp ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(icp_program src/test_icp/icp_program.cpp)
target_link_libraries(icp_program ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(PlannerData src/PlannerData.cpp)
target_link_libraries(PlannerData ${OMPL_LIBRARIES} ${GTSAM_LIBRARIES})

add_executable(pirate_keybord src/pirate_keybord.cpp)
target_link_libraries(pirate_keybord ${BOOST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(pioneerTfPublisher src/pioneerTfPublisher.cpp)
target_link_libraries(pioneerTfPublisher ${catkin_LIBRARIES})

## Declare a C++ library
# add_library(anpl_inf
#   src/${PROJECT_NAME}/anpl_inf.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(anpl_inf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(anpl_inf_node src/anpl_inf_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(anpl_inf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(anpl_inf_node
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
# install(TARGETS anpl_inf anpl_inf_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_anpl_inf.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
