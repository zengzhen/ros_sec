/**
 * \file        view3D.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "view3D.h"
#include <string>

namespace TableObject{
    
//     bool view3D::drawParticles(pcl::visualization::PCLVisualizer& viz, track3D& tracker)
//     {
//         ParticleFilter::PointCloudStatePtr particles = tracker.getParticles ();
//         if (particles && tracker.newCloud())
//         {
//         //Set pointCloud with particle's points
//             pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//             for (size_t i = 0; i < particles->points.size (); i++)
//             {
//                 pcl::PointXYZ point;
// 
//                 point.x = particles->points[i].x;
//                 point.y = particles->points[i].y;
//                 point.z = particles->points[i].z;
//                 particle_cloud->points.push_back (point);
//             }
// 
//             //Draw red particles 
//             {
//                 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color (particle_cloud, 250, 99, 71);
//                 viz.removePointCloud("particle_cloud");
//                 if (!viz.updatePointCloud (particle_cloud, red_color, "particle cloud"))
//                     viz.addPointCloud (particle_cloud, red_color, "particle cloud");
//             }
//             return true;
//         }else
//         {
//             return false;
//         }
//     }

    CloudPtr view3D::drawResult(pcl::visualization::PCLVisualizer& viz, track3D& tracker)
    {

        pcl::tracking::ParticleXYZRPY result = tracker.getResult ();
        Eigen::Affine3f transformation = tracker.toEigenMatrix (result);

        //move close to camera a little for better visualization
        transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
        CloudPtr result_cloud (new Cloud);
        pcl::transformPointCloud<RefPointType> (*(tracker.getReferenceCloud ()), *result_cloud, transformation);

        //Draw blue model reference point cloud
        pcl::visualization::PointCloudColorHandlerCustom<RefPointType> blue_color (result_cloud, 0, 0, 255);
//         viz.removePointCloud("resultcloud");
        if (!viz.updatePointCloud (result_cloud, blue_color, "resultcloud"))
            viz.addPointCloud (result_cloud, blue_color, "resultcloud");
        
        return result_cloud;
    }
    
    void view3D::drawClusters(pcl::visualization::PCLVisualizer& viz, CloudPtr cloud, std::vector< pcl::PointIndices > clusters)
    {
        for (int i = 0; i < clusters.size(); i++) 
        { 
            // Extract the i_th cluster into a new cloud 
            CloudPtr cluster_i (new Cloud); 
            pcl::copyPointCloud (*cloud, clusters[i], *cluster_i); 
            
            // Create a random color 
            pcl::visualization::PointCloudColorHandlerRandom<RefPointType> random_color (cluster_i); 
            
            // color as the same with the cloud
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> color_cloud(cluster_i);

            // Create a unique identifier 
            std::stringstream cluster_id ("cluster"); 
            cluster_id << i; 

            // Add the i_th cluster to the visualizer with a random color and a unique identifier 
            if(!viz.updatePointCloud(cluster_i, random_color, cluster_id.str()))
            {
                
                viz.addPointCloud(cluster_i, random_color, cluster_id.str ()); 
            }
            
//             std::stringstream ss; ss << i;
//             viz.removeText3D(ss.str());
//             viz.addText3D(ss.str(), cloud->points.at(clusters[i].indices[0]),0.1);
        } 
    }

    void view3D::drawText(pcl::visualization::PCLVisualizer& viz, CloudPtr cloud, std::vector< pcl::PointIndices > clusters)
    {
        for (int i = 0; i < clusters.size(); i++) 
        { 
            
            // Extract the i_th cluster into a new cloud 
            CloudPtr cluster_i (new Cloud); 
            pcl::copyPointCloud (*cloud, clusters[i], *cluster_i); 
            
            // Create a unique identifier 
            std::stringstream cluster_id ("cluster"); 
            cluster_id << i; 
            
            if(i==clusters.size()-1 or i==clusters.size()-2) viz.removePointCloud(cluster_id.str());
            
//             std::stringstream ss; ss << i;
//             viz.removeText3D(ss.str());
//             viz.addText3D(ss.str(), cloud->points.at(clusters[i].indices[0]),0.1);
        } 
    }

}
