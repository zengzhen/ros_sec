
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS}  
                    ${OpenCV_INCLUDE_DIRS} 
                    ${PROJECT_SOURCE_DIR}
                    /home/zhen/Documents) 
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARIES}  ${OpenCV_LIBS})

add_library(mainGraph mainGraph.cpp)
add_library(eventChain eventChain.cpp)
add_library(similarityMeasure similarityMeasure.cpp)