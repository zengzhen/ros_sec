cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS} /home/zhen/Documents)
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBS})
add_definitions(${PCL_DEFINITIONS})

add_library(pcd_cloud pcd_cloud.cpp)
add_library(table_obj_seg table_obj_seg.cpp)