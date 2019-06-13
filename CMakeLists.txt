cmake_minimum_required(VERSION 2.8.3)
project(cerebro)

# Catkin components
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  message_generation
)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set( CMAKE_CXX_STANDARD 11 )
find_package(Threads)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV 3 REQUIRED)
#set( CMAKE_CXX_FLAGS "-fpermissive -std=c++11 -O3 -Wall -Wextra -pedantic" )

# Removing the dependence on theia for the production code.
# If you want to compile unit tests, some of those need theia-sfm as dependence.
find_package(Theia REQUIRED)
include_directories(${THEIA_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})


#----------START gperftools
# install : sudo apt-get install google-perftools libgoogle-perftools-dev
# get cmake file: https://raw.githubusercontent.com/vast-io/vast/master/cmake/FindGperftools.cmake
# save to '/usr/local/lib/cmake'
#list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake")
#find_package(Gperftools REQUIRED)
#message( "----- GPERF")
#message( "----- "${GPERFTOOLS_LIBRARIES} )
set (CMAKE_CXX_FLAGS " -Wl,-no-as-needed")
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

#----------END GPERFTOOLS

option(COMPILE_UNIT_TEST "Compile unit tests" OFF) #OFF by default. catkin_make -DCOMPILE_UNIT_TEST=ON will compile it. OFF will skip it


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
add_message_files(
  FILES
    LoopEdge.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   WholeImageDescriptorCompute.srv
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
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
#  LIBRARIES cerebro
#  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


include_directories(
    src/utils/camodocal/include
    src/utils/
    )
FILE(GLOB CamodocalCameraModelSources
        src/utils/camodocal/src/chessboard/Chessboard.cc
        src/utils/camodocal/src/calib/CameraCalibration.cc
        src/utils/camodocal/src/camera_models/Camera.cc
        src/utils/camodocal/src/camera_models/CameraFactory.cc
        src/utils/camodocal/src/camera_models/CostFunctionFactory.cc
        src/utils/camodocal/src/camera_models/PinholeCamera.cc
        src/utils/camodocal/src/camera_models/CataCamera.cc
        src/utils/camodocal/src/camera_models/EquidistantCamera.cc
        src/utils/camodocal/src/camera_models/ScaramuzzaCamera.cc
        src/utils/camodocal/src/sparse_graph/Transform.cc
        src/utils/camodocal/src/gpl/gpl.cc
        src/utils/camodocal/src/gpl/EigenQuaternionParameterization.cc
    )

FILE( GLOB KuseUtilsSources
        src/utils/PoseManipUtils.cpp
        src/utils/RosMarkerUtils.cpp
        src/utils/RawFileIO.cpp
        src/utils/MiscUtils.cpp
        src/utils/Plot2Mat.cpp
        src/utils/CameraGeometry.cpp
        src/utils/PointFeatureMatching.cpp
        src/utils/GMSMatcher/gms_matcher.cpp
)


## Declare a C++ executable

add_executable(
    cerebro_node
        src/cerebro_node.cpp
        src/PinholeCamera.cpp
        src/DataManager.cpp
        src/DataNode.cpp
        src/Cerebro.cpp
        src/Visualization.cpp
        src/ProcessedLoopCandidate.cpp
        src/DlsPnpWithRansac.cpp
        src/HypothesisManager.cpp
        ${KuseUtilsSources}
        ${CamodocalCameraModelSources}
)

# this is needed to indicate this this executable depends on the messages of the pkg.
add_dependencies(cerebro_node cerebro_generate_messages_cpp)


target_link_libraries(cerebro_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  ${CERES_LIBRARIES}
  ${Boost_LIBRARIES}
  ${THEIA_LIBRARIES}

  #${GPERFTOOLS_LIBRARIES}
  /usr/local/lib/libfaiss.so
)

if(COMPILE_UNIT_TEST)
    #add some compilation flags


    ## Unit Tests
    add_executable (
        unittest_termcolor
            src/unittest/unittest_termcolor.cpp
    )

    add_executable (
        unittest_rosservice_client
            src/unittest/unittest_rosservice_client.cpp
    )

    add_executable (
        unittest_camodocal_proj
            src/unittest/unittest_camodocal_proj.cpp
            ${KuseUtilsSources}
            ${CamodocalCameraModelSources}
    )


    add_executable (
        unittest_camera_geom_class_usage
            src/unittest/unittest_camera_geom_class_usage.cpp
            ${KuseUtilsSources}
            ${CamodocalCameraModelSources}
    )


    add_executable (
        unittest_theia
            src/unittest/unittest_theia.cpp
            ${KuseUtilsSources}
            ${CamodocalCameraModelSources}
    )

    add_executable (
        unittest_pose_tester
            src/unittest/unittest_pose_tester.cpp
            ${KuseUtilsSources}
            ${CamodocalCameraModelSources}
            src/DlsPnpWithRansac.cpp
    )


    add_executable (
        unittest_plot2mat
            src/unittest/unittest_plot2mat.cpp
            src/utils/Plot2Mat.cpp
    )


    target_link_libraries(unittest_rosservice_client
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${CMAKE_THREAD_LIBS_INIT}
      ${CERES_LIBRARIES}
    )


    target_link_libraries(unittest_camodocal_proj
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${CMAKE_THREAD_LIBS_INIT}
      ${CERES_LIBRARIES}
    )

    target_link_libraries(unittest_camera_geom_class_usage
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${CMAKE_THREAD_LIBS_INIT}
      ${CERES_LIBRARIES}
    )



    target_link_libraries(unittest_theia
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${CMAKE_THREAD_LIBS_INIT}
      ${CERES_LIBRARIES}
      ${THEIA_LIBRARIES}
    )

    target_link_libraries(unittest_pose_tester
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${CMAKE_THREAD_LIBS_INIT}
      ${CERES_LIBRARIES}
      ${THEIA_LIBRARIES}
    )

    target_link_libraries(unittest_plot2mat
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
    )


else()
    #add some other compilation flags
endif(COMPILE_UNIT_TEST)
unset(COMPILE_UNIT_TEST CACHE) # <---- this is the important!!



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
# install(TARGETS cerebro cerebro_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cerebro.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
