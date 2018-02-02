cmake_minimum_required(VERSION 2.8.3)
project(icp_test)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED COMPONENTS
  #angles
  #pcl_conversions
  #pcl_ros
  #roscpp
  #roslib
  #rospy
  #sensor_msgs
  #std_msgs
  #tf
  ##velodyne
  #velodyne_driver
  #velodyne_msgs
  #velodyne_pointcloud
#)

find_package(catkin REQUIRED COMPONENTS
	#angles
	#pcl_conversions
	#pcl_ros
	#roscpp
	#roslib
	#rospy
	#sensor_msgs
	#std_msgs
	tf
	#velodyne
	velodyne_driver
	velodyne_msgs
	velodyne_pointcloud
	#libpointmatcher_ros
	message_generation
	ecl_threads
	)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
unset(PCL_LIBRARIES CACHE)
unset(Boost_INCLUDE_DIRS CACHE)
unset(PCL_INCLUDE_DIRS CACHE)
unset(PCL_DEFINITIONS CACHE)
#set(Boost_NO_SYSTEM_PATHS TRUE)
#set(Boost_INCLUDE_DIR "/home/avavav/Documents/workspace/INSTALLSPACE/boost_1_47_0/include")
 #find_package(Boost REQUIRED COMPONENTS thread system)
find_package(Boost)
#set(PCL_DIR "/home/avavav/Documents/workspace/INSTALLSPACE/pcl-1.7.2/share/pcl-1.8")
find_package( PCL REQUIRED )

#find_package(velodyne_pointcloud REQUIRED)
#find_package(velodyne_driver REQUIRED)

#find_package(MRPT REQUIRED slam;vision;gui;topography)
#find_package(libpointmatcher REQUIRED)

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
 #add_message_files(
   #DEPENDENCIES
   #sensor_msgs
   #geometry_msgs
   #FILES
#   Message1.msg
#   Message2.msg
 #)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   processICP.srv
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
   sensor_msgs
   geometry_msgs
#   sensor_msgs#   std_msgs#   velodyne_msgs
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
  #CATKIN_DEPENDS libpointmatcher_ros
#  INCLUDE_DIRS include
#  LIBRARIES icp_test
#  CATKIN_DEPENDS angles pcl_conversions pcl_ros roscpp roslib rospy sensor_msgs std_msgs tf velodyne velodyne_driver velodyne_msgs velodyne_pointcloud
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
message(STATUS "${catkin_INCLUDE_DIRS}")
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	#/home/avavav/Documents/catkin_ws_hydro/src/velodyne/velodyne_driver/include
	#/home/avavav/Documents/catkin_ws_hydro/src/velodyne/velodyne_pointcloud/include
	${PROJECT_SOURCE_DIR}/../velodyne/velodyne_driver/include
	${PROJECT_SOURCE_DIR}/../velodyne/velodyne_pointcloud/include
  ${Boost_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	/home/avavav/Downloads/IbeoSDK2.11.1/src
	${libpointmatcher_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/src/toro
	/home/avavav/Documents/workspace/flirtlib/trunk/src

)

## Declare a cpp library
# add_library(icp_test
#   src/${PROJECT_NAME}/icp_test.cpp
# )

#add_library(icp_tools ${PROJECT_SOURCE_DIR}/src/icp_tools.cpp)

link_directories(${PCL_LIBRARY_DIRS} src ${PROJECT_SOURCE_DIR}/src/toro)
add_definitions(${PCL_DEFINITIONS})

#message(STATUS "${PCL_LIBRARIES} ${PCL_DEFINITIONS} ${PCL_LIBRARY_DIRS}")
message(STATUS "${roscpp_LIBRARIES}")
message(STATUS "${catkin_LIBRARIES}")

## Declare a cpp executable
 #add_executable(icp_test_node src/icp_test_node.cpp)
 #target_link_libraries(icp_test_node ${roscpp_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES}
 #target_link_libraries(icp_test_node  ${PCL_LIBRARIES} ${catkin_LIBRARIES}
 #target_link_libraries(icp_test_node  ${PCL_LIBRARIES} /opt/ros/hydro/lib/libtf.so
	 #/opt/ros/hydro/lib/libtf2_ros.so
	 ##/opt/ros/hydro/lib/libroscpp.so
	 #${PROJECT_SOURCE_DIR}/../roscpp/build/devel/lib/libroscpp.so
	 #/opt/ros/hydro/lib/librosconsole.so
	 #/opt/ros/hydro/lib/librostime.so
	 #/opt/ros/hydro/lib/libroscpp_serialization.so
 ##target_link_libraries(icp_test_node ${roscpp_LIBRARIES} #${catkin_LIBRARIES}
#)


#add_executable(ibeo_ecu_online src/ibeo_ecu_online.cpp)
#target_link_libraries(ibeo_ecu_online /home/avavav/Downloads/IbeoSDK2.11.1/libs/libibeosdk_d.so-2.11.1
  #rt
  #${Boost_SYSTEM_LIBRARY}
  #${Boost_THREAD_LIBRARY}
  #${roscpp_LIBRARIES} ${catkin_LIBRARIES}
  #${libpointmatcher_LIBRARIES}
  #)

#target_include_directories(ibeo_ecu_online PRIVATE /home/avavav/Documents/workspace/flirtlib/trunk/src/feature)
#set_target_properties (ibeo_ecu_online PROPERTIES INCLUDE_DIRECTORIES /home/avavav/Documents/workspace/flirtlib/trunk/src/feature)

#add_executable(ibeo_file_read src/ibeo_file_read.cpp)
#target_link_libraries(ibeo_file_read /home/avavav/Downloads/IbeoSDK2.11.1/libs/libibeosdk_d.so-2.11.1
	#rt
	#${Boost_SYSTEM_LIBRARY}
	#${Boost_THREAD_LIBRARY}
	#${roscpp_LIBRARIES} ${catkin_LIBRARIES}
	#${libpointmatcher_LIBRARIES}
	#)

#add_executable(test_node src/test.cpp)
#target_link_libraries(test_node ${MRPT_LIBS} ${Boost_SYSTEM_LIBRARY}
  #${roscpp_LIBRARIES} ${catkin_LIBRARIES}
  #${libpointmatcher_LIBRARIES}
  #libtoro.a
  #)


#add_executable(test_lpm_node src/test_lpm.cpp)
#target_link_libraries(test_lpm_node ${MRPT_LIBS} ${Boost_SYSTEM_LIBRARY}
  #${roscpp_LIBRARIES} ${catkin_LIBRARIES}
  #${libpointmatcher_LIBRARIES}
  #libtoro.a
  #/home/avavav/Documents/workspace/flirtlib/trunk/build/src/feature/libfeature.so
  #/home/avavav/Documents/workspace/flirtlib/trunk/build/src/utils/libutils.so
  #/home/avavav/Documents/workspace/flirtlib/trunk/build/src/geometry/libgeometry.so
  #)

#add_executable(points_saver_node src/points_saver_node.cpp)
#target_link_libraries(points_saver_node ${MRPT_LIBS} ${Boost_SYSTEM_LIBRARY}
  #${roscpp_LIBRARIES} ${catkin_LIBRARIES}
  #${libpointmatcher_LIBRARIES}
  #libtoro.a
  #)

#add_executable(points_saver_with_filter_node src/points_saver_with_filter_node.cpp)
#target_link_libraries(points_saver_with_filter_node ${MRPT_LIBS} ${Boost_SYSTEM_LIBRARY}
  #${roscpp_LIBRARIES} ${catkin_LIBRARIES}
  #${libpointmatcher_LIBRARIES}
  #libtoro.a
  #)


#add_executable(online_icp_node src/online_icp_node.cpp)
#target_link_libraries(online_icp_node ${MRPT_LIBS} ${Boost_SYSTEM_LIBRARY}
  #${roscpp_LIBRARIES} ${catkin_LIBRARIES}
  #${libpointmatcher_LIBRARIES}
  #libtoro.a
  #icp_tools
  #)

#add_executable(online_second_icp src/online_second_icp.cpp)
#target_link_libraries(online_second_icp ${MRPT_LIBS} ${Boost_SYSTEM_LIBRARY}
  #${roscpp_LIBRARIES} ${catkin_LIBRARIES}
  #${libpointmatcher_LIBRARIES}
  #libtoro.a
  #icp_tools
  #)

#add_executable(online_lpm_node src/online_lpm_node.cpp)
#target_link_libraries(online_lpm_node ${MRPT_LIBS} ${Boost_SYSTEM_LIBRARY}
  #${roscpp_LIBRARIES} ${catkin_LIBRARIES}
  #${libpointmatcher_LIBRARIES}
  #libtoro.a
  #icp_tools
  #)


#add_executable(online_second_lpm src/online_second_lpm.cpp)
#target_link_libraries(online_second_lpm ${MRPT_LIBS} ${Boost_SYSTEM_LIBRARY}
  #${roscpp_LIBRARIES} ${catkin_LIBRARIES}
  #${libpointmatcher_LIBRARIES}
  #libtoro.a
  #icp_tools
  #)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(icp_test_node icp_test_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(icp_test_node
#   ${catkin_LIBRARIES}
# )

add_executable(velosync_node src/velosync.cpp)
target_link_libraries(velosync_node  #${Boost_SYSTEM_LIBRARY}
	#${roscpp_LIBRARIES} 
	${catkin_LIBRARIES}
)

add_executable(velosync_3_node src/velosync_3.cpp)
target_link_libraries(velosync_3_node  #${Boost_SYSTEM_LIBRARY}
	#${roscpp_LIBRARIES} 
	${catkin_LIBRARIES}
)

add_executable(velosync_3_fixedout_node src/velosync_3_fixedout.cpp)
target_link_libraries(velosync_3_fixedout_node  #${Boost_SYSTEM_LIBRARY}
  #${roscpp_LIBRARIES} 
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
# install(TARGETS icp_test icp_test_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_icp_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
