cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR} /home/zhen/Documents) 
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARIES}  ${OpenCV_LIBS})

add_library(track3D track3D.cpp)
add_library(trackRigid trackRigid.cpp)