cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS}  
                    ${OpenCV_INCLUDE_DIRS} 
                    ${PROJECT_SOURCE_DIR}
                    /home/zhen/Documents) 
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARIES}  ${OpenCV_LIBS})

add_executable (extract_sec extract_sec.cpp)
target_link_libraries (extract_sec 
                       ${PCL_LIBRARIES} 
                       ${OpenCV_LIBS}
                       /home/zhen/Documents/Kinect/lib/libconvert_rgbd_to_image.so
                       table_obj_seg
                       pcd_cloud
                       trackRigid
                       view2D
                       view3D
                       util
                       colorDetector
                       touchDetecor
                       moveDetector
                       mainGraph)

                       
add_executable (testEventChain testEventChain.cpp)
target_link_libraries (testEventChain 
                       eventChain)
                       
add_executable (testSimilarityMeasure testSimilarityMeasure.cpp)
target_link_libraries (testSimilarityMeasure
                       similarityMeasure
                       eventChain)
                       
add_executable (actionClustering actionClustering.cpp)
target_link_libraries (actionClustering
                       similarityMeasure
                       eventChain)
                       
add_executable (actionClassification actionClassification.cpp)
target_link_libraries (actionClassification
                       similarityMeasure
                       eventChain)
                       
add_executable (display_pcd display_pcd.cpp)
target_link_libraries (display_pcd
                       ${PCL_LIBRARIES} 
                       ${OpenCV_LIBS}
                       /home/zhen/Documents/Kinect/lib/libconvert_rgbd_to_image.so
                       table_obj_seg
                       pcd_cloud
                       trackRigid
                       view2D
                       view3D
                       util
                       colorDetector
                       touchDetecor
                       moveDetector
                       mainGraph)
