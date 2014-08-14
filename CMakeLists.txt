cmake_minimum_required(VERSION 2.8.3)
project(sec)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
#   pcl
  roscpp
)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

set(OpenCV_INCLUDE_DIRS /usr/local/include;/usr/local/include/opencv)
set(OpenCV_LIBS /usr/local/lib/libopencv_calib3d.so /usr/local/lib/libopencv_contrib.so /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_features2d.so /usr/local/lib/libopencv_flann.so /usr/local/lib/libopencv_gpu.so /usr/local/lib/libopencv_highgui.so /usr/local/lib/libopencv_imgproc.so /usr/local/lib/libopencv_legacy.so /usr/local/lib/libopencv_ml.so /usr/local/lib/libopencv_nonfree.so /usr/local/lib/libopencv_objdetect.so /usr/local/lib/libopencv_photo.so /usr/local/lib/libopencv_stitching.so /usr/local/lib/libopencv_ts.so /usr/local/lib/libopencv_video.so /usr/local/lib/libopencv_videostab.so)

message ( "\${OpenCV_INCLUDE_DIRS} = ${OpenCV_INCLUDE_DIRS}")
message ( "\${OpenCV_LIBS} = ${OpenCV_LIBS}")

message( "\${catkin_INCLUDE_DIRS} = ${catkin_INCLUDE_DIRS}" )
message( "\${catkin_LIBRARIES} = ${catkin_LIBRARIES}" )

message ( "\${PCL_LIBRARIES} = ${PCL_LIBRARIES}")
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
#   std_msgs  # Or other packages containing msgs
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
#     INCLUDE_DIRS include
    LIBRARIES sec
    CATKIN_DEPENDS roscpp
    DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    /home/kuipers
    include
    include/sec
    include/sec/Detector
    include/sec/TableObjectSegmentation
    include/sec/Tracker
    include/sec/util
    include/sec/Visualizer
    include/sec/SEC
)
add_definitions(${PCL_DEFINITIONS})
# link_directories(${PCL_LIBRARIES}  ${OpenCV_LIBS})

## Declare a cpp library
add_library(pcd_cloud src/TableObjectSegmentation/pcd_cloud.cpp)
add_library(table_obj_seg src/TableObjectSegmentation/table_obj_seg.cpp)

add_library(touchDetecor src/Detector/touchDetector.cpp)
add_library(colorDetector src/Detector/colorDetector.cpp)
add_library(moveDetector src/Detector/moveDetector.cpp)

add_library(view2D src/Visualizer/view2D.cpp)
add_library(view3D src/Visualizer/view3D.cpp)

add_library(track3D src/Tracker/track3D.cpp)
add_library(trackRigid src/Tracker/trackRigid.cpp)

add_library(util src/util/util.cpp)

add_library(mainGraph src/SEC/mainGraph.cpp)
# add_library(eventChain src/SEC/eventChain.cpp)
# add_library(similarityMeasure src/SEC/similarityMeasure.cpp)

## Declare a cpp executable
add_executable(extract_sec src/tools/extract_sec.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(SEC_node SEC_generate_messages_cpp)

target_link_libraries(
    view2D
    ${OpenCV_LIBS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(extract_sec
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    /home/kuipers/Kinect/lib/libconvert_rgbd_to_image.so
    table_obj_seg
    pcd_cloud
    trackRigid
    view2D
    view3D
    util
    colorDetector
    touchDetecor
    moveDetector
    mainGraph
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
# install(TARGETS SEC SEC_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_SEC.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
