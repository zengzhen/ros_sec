/**
 * \file        extract_sec.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "TableObjectSegmentation/table_obj_seg.h"
#include "TableObjectSegmentation/pcd_cloud.h"
#include "Visualizer/view2D.h"
#include "Visualizer/view3D.h"

#include <sys/stat.h>

bool VERBOSE = false;
bool DEBUG = false;
bool DEBUG_COLOR = false;

int
main (int argc, char** argv)
{
    pcl::visualization::PCLVisualizer result_viewer("planar_segmentation");
    result_viewer.addCoordinateSystem();
    result_viewer.setCameraPosition(-0.504623,0.0647437,-0.758519, -0.443141,0.0788583,-0.502855, 0.00533475,-0.998535,0.0538437);
    result_viewer.setCameraClipDistances(0.0136198,13.6198);
    //2.8494,8.48591/0.0712695,0.486438,0.865/-1.77289,1.38452,-4.06431/0.0224361,-0.982044,-0.187315/0.523599/1920,600/1921,52
    //clipDistance  / pos x, y, z              / view x, y, z            / up x, y, z                  / fovy   /win_size/win_pos
    //0.0136198,13.6198/-0.443141,0.0788583,-0.502855/-0.504623,0.0647437,-0.758519/0.00533475,-0.998535,0.0538437/0.523599/800,450/425,277
    
    /***************************************
    *  parse arguments
    ***************************************/
    if(argc<5)
    {
        std::cerr << "Usage: display_pcd DATA_PATH/PCD_FILE_FORMAT DEMO_NAME FRAME_INDEX FRAME_NAME" << std::endl;
        exit(1);
    }
    
    std::string basename_cloud=argv[1];
    std::string demo_name=argv[2];
    unsigned int idx = std::atoi(argv[3]);
    std::string frame_name=argv[4];
    
    /***************************************
    *  set up result directory
    ***************************************/
    mkdir("../../../result", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    char result_folder[50];
    std::snprintf(result_folder, sizeof(result_folder), "../../../result/%s", demo_name.c_str());
    mkdir(result_folder, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    
    std::string basename_pcd = (basename_cloud.find(".pcd") == std::string::npos) ? (basename_cloud + ".pcd") : basename_cloud;
    std::string filename_pcd;
    
    TableObject::Segmentation initialSeg;
    TableObject::pcdCloud pcdSceneCloud;
    CloudPtr sceneCloud;
    
    initialSeg.setThreshold(200);
    
    /***************************************
    *  start processing
    ***************************************/
    std::cout << std::endl;
    std::cout << "frame id=" << idx << std::endl;
    filename_pcd = cv::format(basename_cloud.c_str(), idx);
        
    /***************************************
        *  object cloud extraction
        ***************************************/
    initialSeg.resetCloud(filename_pcd);
    initialSeg.seg(false);
    initialSeg.getsceneCloud(pcdSceneCloud);
    sceneCloud=pcdSceneCloud.getCloud();
    
    /***************************************
    *  Visualization
    ***************************************/  
    // darw original cloud
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(sceneCloud);
    result_viewer.addPointCloud<RefPointType>(sceneCloud, rgb, "new frame");
    
    result_viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    
    char screenshot[100]; // make sure it's big enough
    std::snprintf(screenshot, sizeof(screenshot), "../../../result/%s/%s.png", demo_name.c_str(), frame_name.c_str());
    result_viewer.saveScreenshot(screenshot);
    std::cout << screenshot << std::endl;


        
    while (!result_viewer.wasStopped ())
    {
        result_viewer.spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
}
