#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_grabber.h>

#include "TableObjectSegmentation/table_obj_seg.h"
#include "TableObjectSegmentation/pcd_cloud.h"
#include "Tracker/track3D.h"
#include "Visualizer/view2D.h"
#include "Visualizer/view3D.h"

#include "typeDef.h"

int obj_id=-1;
TableObject::Segmentation tableObjSegTest;
TableObject::track3D tracker;
pcl::visualization::PCLVisualizer result_viewer("planar_segmentation");
pcl::visualization::CloudViewer* viewer_ = new pcl::visualization::CloudViewer("PCL OpenNI Tracking Viewer");
boost::mutex updateModelMutex;
CloudPtr cloud_hull(new Cloud);
int counter=0;
CloudPtr cloud_pass;
boost::mutex mtx_;

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

void viz_cb (pcl::visualization::PCLVisualizer& viz)
{
    boost::mutex::scoped_lock lock (updateModelMutex); //mtx_
    
    if (!cloud_pass)
    {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
        return;
    }

    if (!viz.updatePointCloud (cloud_pass, "cloudpass"))
    {
        viz.addPointCloud (cloud_pass, "cloudpass");
//         viz.resetCameraViewpoint ("cloudpass");
    }
   
        
//     if (tracker.newCloud() && cloud_pass)
//     {
//         if (!viz.updatePointCloud (cloud_pass, "cloudpass"))
//         {
//             viz.addPointCloud (cloud_pass, "cloudpass");
//         }
//         bool ret=TableObject::view3D::drawParticles(viz, tracker);
//         if (ret) 
//             TableObject::view3D::drawResult(viz, tracker);
//         
//     }
//     tracker.setNewCloud(false);
}

void cloud_cb (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
{
    boost::mutex::scoped_lock lock (mtx_);
    
    if (!viewer_->wasStopped())
    {
        viewer_->showCloud (cloud);
    }
    
//     cloud_pass.reset(new Cloud);
//     pcl::copyPointCloud(*cloud, *cloud_pass);
//     /*************************************************
//     * Table Object Segmentation
//     *************************************************/
//     CloudPtr cloud_objects(new Cloud);
//     std::vector<pcl::PointIndices> clusters;
//     CloudPtr tempcloud(new Cloud);
//     pcl::copyPointCloud(*cloud, *tempcloud);
//     
//     tableObjSegTest.resetCloud(tempcloud);
//     //             TableObject::Segmentation tableObjSeg(filename_pcd);
//     tableObjSegTest.setThreshold(400);
//     tableObjSegTest.seg(false);
//     tableObjSegTest.getObjects(cloud_objects, clusters);
//     tableObjSegTest.getCloudHull(cloud_hull);
//     if(obj_id<0) 
//     {
//         TableObject::view3D::drawClusters(result_viewer, cloud_objects, clusters);
// //         result_viewer.addPolygon<RefPointType>(cloud_hull, 0, 255, 0, "polygon");
//     }
//     
//     cloud_pass=cloud_objects;
// 
//   if(counter < 10){
//     counter++;
//   }else if(obj_id>0)
//   {
//     //Track the object
//     tracker.track(cloud_objects);
//   }
}

int main (int argc, char** argv)
{
    result_viewer.addCoordinateSystem();
//     result_viewer.setCameraPosition(-1.77289,1.38452,-4.06431, 0.0712695,0.486438,0.865, 0.0224361,-0.982044,-0.187315);
//     result_viewer.setCameraClipDistances(2.8494,8.48591);
    //2.8494,8.48591/0.0712695,0.486438,0.865/-1.77289,1.38452,-4.06431/0.0224361,-0.982044,-0.187315/0.523599/1920,600/1921,52
    //clipDistance  / pos x, y, z              / view x, y, z            / up x, y, z                  / fovy   /win_size/win_pos
//     result_viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&result_viewer);
   
    obj_id=std::atoi(argv[1]);
    
    if(obj_id<0)
    {
        std::cout << "entering preview state..." << std::endl;
//         pcl::visualization::CloudViewer* viewer_ = new pcl::visualization::CloudViewer("PCL OpenNI Tracking Viewer");
        pcl::Grabber* interface = new pcl::OpenNIGrabber ();
        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
            boost::bind (&cloud_cb, _1);
        interface->registerCallback (f);
        
//         viewer_->runOnVisualizationThread (boost::bind(&viz_cb, _1), "viz_cb");

        //Start viewer
        interface->start();
//         while (!result_viewer.wasStopped ())
//         {
//             result_viewer.spinOnce (100);
//             boost::this_thread::sleep(boost::posix_time::seconds(1));
//         }
        boost::this_thread::sleep(boost::posix_time::seconds(1));
        std::cin.get();
        interface->stop();
        
    }else
    {  
//         pcl::visualization::CloudViewer* viewer_ = new pcl::visualization::CloudViewer("PCL OpenNI Tracking Viewer");
        pcl::Grabber* interface = new pcl::OpenNIGrabber ();
        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
            boost::bind (&cloud_cb,  _1);
        interface->registerCallback (f);
        
        viewer_->registerKeyboardCallback(keyboardEventOccurred); 
        viewer_->runOnVisualizationThread (boost::bind(&viz_cb, _1), "viz_cb");
        
        //Start viewer
        interface->start();
        while (!viewer_->wasStopped ())
            boost::this_thread::sleep(boost::posix_time::seconds(1));
        interface->stop();
        
    }
    
    

    return (0);
  
}
