#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>

#include "TableObjectSegmentation/table_obj_seg.h"
#include "TableObjectSegmentation/pcd_cloud.h"
#include "Tracker/track3D.h"
#include "Visualizer/view2D.h"
#include "Visualizer/view3D.h"

int obj_id=-1;
TableObject::Segmentation tableObjSegTest;
TableObject::track3D tracker;
pcl::visualization::PCLVisualizer result_viewer("planar_segmentation");
boost::mutex updateModelMutex;

void assignTracker(ParticleFilter& _tracker)
{
    boost::shared_ptr<pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
        (new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8));

    ParticleT bin_size;
    bin_size.x = 0.1f;
    bin_size.y = 0.1f;
    bin_size.z = 0.1f;
    bin_size.roll = 0.1f;
    bin_size.pitch = 0.1f;
    bin_size.yaw = 0.1f;
    
    //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
    tracker->setMaximumParticleNum (1000);
    tracker->setDelta (0.99);
    tracker->setEpsilon (0.2);
    tracker->setBinSize (bin_size);
    //Set all parameters for  ParticleFilter
    _tracker = *tracker;
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    if (event.keyDown ())
    {
        std::cout << event.getKeySym() << " was pressed" << std::endl;
        if(event.getKeySym()=="q")
            viewer->close();
        else if(event.getKeySym()=="o")
        {
            std::cout << "input the object id for tracking" << std::endl;
            std::cin >> obj_id;
//             std::string sName(reinterpret_cast<char*>(event.getKeyCode()));
//             std::istringstream convert(sName);
//             convert>>obj_id;
            std::cout << "recording #" << obj_id << " cluster..." << std::endl;
//             obj_id=std::atoi(event.getKeyCode());
            tableObjSegTest.recordTarget(obj_id);
        }
    }
}

void viz_cb ()
{
    while (!result_viewer.wasStopped ())
    {
        boost::mutex::scoped_lock lock (updateModelMutex); //mtx_
        //Draw downsampled point cloud from sensor    
        if (tracker.newCloud())
        {
            bool ret=TableObject::view3D::drawParticles(result_viewer, tracker);
            if (ret) 
                TableObject::view3D::drawResult(result_viewer, tracker);
            tracker.setNewCloud(false);
        }
        lock.unlock();
        result_viewer.spinOnce (100);
    }
}

int main (int argc, char** argv)
{
    result_viewer.addCoordinateSystem();
    result_viewer.setCameraPosition(-1.77289,1.38452,-4.06431, 0.0712695,0.486438,0.865, 0.0224361,-0.982044,-0.187315);
    result_viewer.setCameraClipDistances(2.8494,8.48591);
    //2.8494,8.48591/0.0712695,0.486438,0.865/-1.77289,1.38452,-4.06431/0.0224361,-0.982044,-0.187315/0.523599/1920,600/1921,52
    //clipDistance  / pos x, y, z              / view x, y, z            / up x, y, z                  / fovy   /win_size/win_pos
    result_viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&result_viewer);
    
    std::string basename_cloud=argv[1];
    unsigned int index_start = std::atoi(argv[2]);
    unsigned int index_end = std::atoi(argv[3]);
    obj_id=std::atoi(argv[4]);
    
    std::string basename_pcd = (basename_cloud.find(".pcd") == std::string::npos) ? (basename_cloud + ".pcd") : basename_cloud;
    std::string filename_pcd;
    
    unsigned int idx = index_start;
    bool is_video_started = false;
   
    CloudPtr cloud_hull(new Cloud);
    CloudPtr cluster_ref (new Cloud);
    
    if(obj_id<0)
    {
        std::cout << "frame id=" << idx << std::endl;
        filename_pcd = cv::format(basename_cloud.c_str(), idx);
        
        if( boost::filesystem::exists( filename_pcd ) )
        {
            /*************************************************
            * Table Object Segmentation
            *************************************************/
            CloudPtr cloud_objects(new Cloud);
            std::vector<pcl::PointIndices> clusters;
            tableObjSegTest.resetCloud(filename_pcd);
//             TableObject::Segmentation tableObjSeg(filename_pcd);
            tableObjSegTest.setThreshold(400);
            
            tableObjSegTest.seg(true);
            tableObjSegTest.getObjects(cloud_objects, clusters);
            tableObjSegTest.getCloudHull(cloud_hull);
            
            
            /*************************************************
            * Visualization
            *************************************************/
            TableObject::view3D::drawClusters(result_viewer, cloud_objects, clusters);
            result_viewer.addPolygon<RefPointType>(cloud_hull, 0, 255, 0, "polygon");
        }
    }else
    {
//         boost::thread workerThread(viz_cb);
        while( idx <= index_end && !result_viewer.wasStopped())
        {            
            std::cout << "frame id=" << idx << std::endl;
            filename_pcd = cv::format(basename_cloud.c_str(), idx);
            
            if( boost::filesystem::exists( filename_pcd ) )
            {
                /*************************************************
                * Table Object Segmentation
                *************************************************/
                CloudPtr cloud_objects(new Cloud);
                std::vector<pcl::PointIndices> clusters;
                
                TableObject::Segmentation tableObjSeg(filename_pcd);
                tableObjSeg.setThreshold(400);
                if(idx==index_start)
                {
                    is_video_started = true;
                    tableObjSeg.seg(false);
                }else
                {
                    tableObjSeg.seg(cloud_hull, false);
                }
                tableObjSeg.getObjects(cloud_objects, clusters);
                tableObjSeg.getCloudHull(cloud_hull);
                
                /*************************************************
                * Tracking
                *************************************************/
                if(idx==index_start) 
                {
//                     CloudPtr cluster_ref (new Cloud);
                    pcl::copyPointCloud (*cloud_objects, clusters[obj_id], *cluster_ref);
                    assignTracker(tracker);
                    tracker.setTarget(cluster_ref);
                    tracker.initialize();
                }else
                {
                    TableObject::pcdCloud sceneCloud;
                    tableObjSeg.getsceneCloud(sceneCloud);
                    tracker.track(sceneCloud.getCloud());
//                     if(cloud_objects->isOrganized())
//                         std::cout << "cloud_objects is organized \n";
//                     tracker.track(cloud_objects);
                    
                    std::cout << "successfully tracked at frame " << idx <<  std::endl;
                    
                    
                    bool ret=TableObject::view3D::drawParticles(result_viewer, tracker);
                    if (ret) 
                    {
                        TableObject::view3D::drawResult(result_viewer, tracker);
                    }
                }
                
                /*************************************************
                * Visualization
                *************************************************/
            //     pcl::visualization::PointCloudColorHandlerCustom<RefPointType> plane_color(tableTopCloud, 0, 0, 255);    
            //     result_viewer.addPointCloud<RefPointType>(tableTopCloud, "plane");
                result_viewer.addPolygon<RefPointType>(cloud_hull, 0, 255, 0, "polygon");
                
                //visualize tracking result
                if (!result_viewer.updatePointCloud (cloud_objects, "object"))
                    result_viewer.addPointCloud (cloud_objects, "object");
               
                result_viewer.spinOnce (100);
                
            //     TableObject::view3D::drawClusters(result_viewer, cloud_objects, clusters);
            //     result_viewer.setRepresentationToSurfaceForAllActors();
                
            }
            else if( is_video_started )
            {
                std::cout<<"End of the video.\n";
                break;
            }
            
            idx=idx+1;
        }
//         workerThread.join(); 
    }
    
    while (!result_viewer.wasStopped ())
    {
        result_viewer.spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return (0);
  
}
