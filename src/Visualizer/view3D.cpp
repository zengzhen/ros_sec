/**
 * \file        view3D.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "ros_sec/Visualizer/view3D.h"
#include <string>

namespace TableObject{
    
    void view3D::drawClusters(pcl::visualization::PCLVisualizer& viz, CloudPtr cloud, std::vector< pcl::PointIndices > clusters, bool showText)
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
            
            if(showText)
            {
                std::stringstream ss; ss << i;
                viz.removeText3D(ss.str());
                viz.addText3D(ss.str(), cloud->points.at(clusters[i].indices[0]),0.1);
            }
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
    
    void view3D::drawArrow(pcl::visualization::PCLVisualizer& viz, pcl::PointXYZ point, Eigen::Vector3f orientation, std::string id)
    {
        pcl::PointXYZ arrow(point.x+orientation[0]*0.3, point.y+orientation[1]*0.3, point.z+orientation[2]*0.3);
        viz.removeShape(id);
        viz.addArrow(arrow, point, 1, 0, 0, 0, id);
    }
    

}
