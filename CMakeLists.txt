cmake_minimum_required(VERSION 2.8.3)
project(ros_sec)

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

# message ( "\${OpenCV_INCLUDE_DIRS} = ${OpenCV_INCLUDE_DIRS}")
# message ( "\${OpenCV_LIBS} = ${OpenCV_LIBS}")
# 
# message( "\${catkin_INCLUDE_DIRS} = ${catkin_INCLUDE_DIRS}" )
# message( "\${catkin_LIBRARIES} = ${catkin_LIBRARIES}" )

set(PCL_BOOST_LIB_DIR /home/kuipers/boost_1_48_0/lib)
set(PCL_INCLUDE_DIRS /usr/include/pcl-1.7;/usr/include/eigen3;/usr/include/ni;/usr/include/vtk-5.8;/usr/include/qhull)
set(PCL_LIBRARIES ${PCL_BOOST_LIB_DIR}/libboost_system-mt.so;${PCL_BOOST_LIB_DIR}/libboost_filesystem-mt.so;
${PCL_BOOST_LIB_DIR}/libboost_thread-mt.so;${PCL_BOOST_LIB_DIR}/libboost_date_time-mt.so;${PCL_BOOST_LIB_DIR}/libboost_iostreams-mt.so;
${PCL_BOOST_LIB_DIR}/libboost_serialization-mt.so;${PCL_BOOST_LIB_DIR}/libboost_chrono-mt.so;
/usr/lib/libpcl_common.so;debug;/usr/lib/libpcl_common.so;optimized;
/usr/lib/libpcl_octree.so;debug;/usr/lib/libpcl_octree.so;/usr/lib/libOpenNI.so;
vtkCommon;vtkFiltering;vtkImaging;vtkGraphics;vtkGenericFiltering;vtkIO;vtkRendering;vtkVolumeRendering;vtkHybrid;
vtkWidgets;vtkParallel;vtkInfovis;vtkGeovis;vtkViews;vtkCharts;optimized;
/usr/lib/libpcl_io.so;debug;/usr/lib/libpcl_io.so;optimized;/opt/ros/groovy/lib/libflann_cpp_s.a;debug;/opt/ros/groovy/lib/libflann_cpp_s-gd.a;optimized;
/usr/lib/libpcl_kdtree.so;debug;/usr/lib/libpcl_kdtree.so;optimized;/usr/lib/libpcl_search.so;debug;/usr/lib/libpcl_search.so;optimized;
/usr/lib/libpcl_sample_consensus.so;debug;/usr/lib/libpcl_sample_consensus.so;optimized;/usr/lib/libpcl_filters.so;debug;
/usr/lib/libpcl_filters.so;optimized;/usr/lib/libpcl_features.so;debug;/usr/lib/libpcl_features.so;optimized;/usr/lib/libpcl_keypoints.so;debug;
/usr/lib/libpcl_keypoints.so;optimized;/usr/lib/libqhull.so;debug;/usr/lib/libqhull.so;optimized;/usr/lib/libpcl_surface.so;debug;
/usr/lib/libpcl_surface.so;optimized;/usr/lib/libpcl_registration.so;debug;/usr/lib/libpcl_registration.so;optimized;/usr/lib/libpcl_ml.so;debug;
/usr/lib/libpcl_ml.so;optimized;/usr/lib/libpcl_segmentation.so;debug;/usr/lib/libpcl_segmentation.so;optimized;/usr/lib/libpcl_recognition.so;debug;
/usr/lib/libpcl_recognition.so;optimized;/usr/lib/libpcl_visualization.so;debug;/usr/lib/libpcl_visualization.so;optimized;/usr/lib/libpcl_people.so;debug;
/usr/lib/libpcl_people.so;optimized;/usr/lib/libpcl_outofcore.so;debug;/usr/lib/libpcl_outofcore.so;optimized;/usr/lib/libpcl_stereo.so;debug;
/usr/lib/libpcl_stereo.so;optimized;/usr/lib/libpcl_tracking.so;debug;/usr/lib/libpcl_tracking.so;/usr/lib/libqhull.so;debug;/usr/lib/libqhull.so;
/usr/lib/libOpenNI.so;optimized;/opt/ros/groovy/lib/libflann_cpp_s.a;debug;/opt/ros/groovy/lib/libflann_cpp_s-gd.a;
vtkCommon;vtkFiltering;vtkImaging;vtkGraphics;vtkGenericFiltering;vtkIO;vtkRendering;vtkVolumeRendering;vtkHybrid;vtkWidgets;vtkParallel;
vtkInfovis;vtkGeovis;vtkViews;vtkCharts
)

# message( "\${PCL_INCLUDE_DIRS} = ${PCL_INCLUDE_DIRS}" )
# message ( "\${PCL_LIBRARIES} = ${PCL_LIBRARIES}")

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
    INCLUDE_DIRS include
    LIBRARIES ros_sec
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
)
add_definitions(${PCL_DEFINITIONS})
# link_directories(${PCL_LIBRARIES}  ${OpenCV_LIBS})

## Declare a cpp library
add_library(pcd_cloud src/TableObjectSegmentation/pcd_cloud.cpp)
add_library(table_obj_seg src/TableObjectSegmentation/table_obj_seg.cpp)

add_library(touchDetector src/Detector/touchDetector.cpp)
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
add_library(ros_sec src/tools/extract_sec.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(SEC_node SEC_generate_messages_cpp)
target_link_libraries(
    pcd_cloud
    ${PCL_LIBRARIES}
)

target_link_libraries(
    table_obj_seg
    pcd_cloud
    view2D
    util
)

target_link_libraries(
    touchDetector
    pcd_cloud
    util
)

target_link_libraries(
    colorDetector
    view2D
)

target_link_libraries(
    moveDetector
    util
)

target_link_libraries(
    view2D
    ${OpenCV_LIBS}
    /home/kuipers/Kinect/lib/libconvert_rgbd_to_image.so
)

target_link_libraries(
    view3D
    track3D
)

target_link_libraries(
    track3D
    ${PCL_LIBRARIES}
)

target_link_libraries(
    trackRigid
    ${PCL_LIBRARIES}
)

target_link_libraries(
    util
    ${PCL_LIBRARIES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(ros_sec
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    
    table_obj_seg
    pcd_cloud
    trackRigid
    view2D
    view3D
    util
    colorDetector
    touchDetector
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
