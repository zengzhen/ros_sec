/**
 * \file        touchDetector.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "touchDetector.h"
#include "util/util.h"
#include <pcl/segmentation/extract_polygonal_prism_data.h>

namespace TableObject{
    
    touchDetector::touchDetector(double threshold)
    :_threshold(threshold){}
    
    bool touchDetector::detect(const CloudPtr& cloud_1, const CloudPtr& cloud_2)
    {
        std::vector<float> distance;
        
        // kdtree initialization: k nearest neighbor search
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        pcl::PointCloud<pcl::PointXYZ>::Ptr kd_tree_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_2, *kd_tree_cloud);
        kdtree.setInputCloud(kd_tree_cloud);
        //viewer.addPointCloud<pcl::PointXYZ>(kd_tree_cloud, "debug object");
        //viewer.addPointCloud<pcl::PointXYZRGBA>(cloud_1, "debug finger");
        int K = 1;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);    

        for(int j=0; j<cloud_1->points.size(); j++)
        {
            pcl::PointXYZ ite;
            ite.x = cloud_1->points[j].x;
            ite.y = cloud_1->points[j].y;
            ite.z = cloud_1->points[j].z;
            
            kdtree.nearestKSearch(ite, K, pointIdxNKNSearch, pointNKNSquaredDistance);
            distance.push_back(sqrt(pointNKNSquaredDistance[0]));
        }
        
        std::sort(distance.begin(), distance.end()); // from small to large
        
//         if(VERBOSE) 
            std::cout<< "nearest distance = " << distance[0] << std::endl;
        if(distance[0] <= _threshold) 
        {
            _touch = true;
            return true;
        }else{
            _touch = false;
            return false;
        }
    }

    bool touchDetector::detectTableTouch(const CloudPtr& cloud_1, pcl::ModelCoefficients coefficients)
    {
        std::vector<float> distance;
        pcl::PointXYZ point;
        
        for(int i=0; i<cloud_1->points.size(); i++)
        {
            point.x = cloud_1->points[i].x;
            point.y = cloud_1->points[i].y;
            point.z = cloud_1->points[i].z;
            float distance_i=pcl::pointToPlaneDistance<pcl::PointXYZ>(point, coefficients.values[0], coefficients.values[1], coefficients.values[2], coefficients.values[3]);
            distance.push_back(distance_i);
        }
        std::sort(distance.begin(), distance.end()); // from small to large
        std::cout<< "nearest distance = " << distance[0] << std::endl;
        if(distance[0] <= 0.02)
        {
            _touch = true;
            return true;
        }else{
            _touch = false;
            return false;
        }

    }
    
    void touchDetector::showTouch(pcl::visualization::PCLVisualizer& viewer, std::string pair, int x, int y)
    {
        viewer.removeShape(pair);
        std::string touch_phrase = pair + ": T";
        std::string ntouch_phrase = pair + ": NT";
        if(_touch)
        {
            viewer.addText(touch_phrase, x,y,20, 0,1,0,pair);
        }else{
            viewer.addText(ntouch_phrase, x,y,20, 1,0,0,pair);
        }
    }



}
