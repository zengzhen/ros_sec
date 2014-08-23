/**
 * \file        util.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include <pcl/common/centroid.h>
#include "ros_sec/util/util.h"
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "ros_sec/Detector/touchDetector.h"

namespace TableObject{
    
    void computeObjCentroid(CloudPtr cloud, std::vector<pcl::PointIndices> clusters, std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> >& centroid)
    {
        for (int i = 0; i < clusters.size(); i++) 
        {  
            Eigen::Vector4f centroid_i;
            pcl::compute3DCentroid<RefPointType>(*cloud, clusters[i], centroid_i);
            centroid.push_back(centroid_i);
            
            if(VERBOSE) std::cout<<"centroid_i: x=" << centroid_i[0] << " y=" << centroid_i[1] << " z=" << centroid_i[2] << " ?=" << centroid_i[3] << std::endl;
        } 
    }

    pcl::PointXYZ computeObjCentroid(CloudPtr cloud)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid<RefPointType>(*cloud, centroid);
        pcl::PointXYZ result;
        result.x = centroid[0];
        result.y = centroid[1];
        result.z = centroid[2];
        
        return result;
    }
    
    linkerList linking(CloudPtr input_cloud, const std::vector< pcl::PointIndices >& input_clusters, CloudPtr target_cloud, const std::vector< pcl::PointIndices >& target_clusters)
    {
        linkerList result;
        
        // compute centroids of input_cloud
        std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > centroid;
        computeObjCentroid(input_cloud, input_clusters, centroid);
        
        // cloud_xyz: convert target_cloud to pointXYZ
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        
        // kdtree initialization: Neighbors within radius search
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        
        for(int i=0;i<input_clusters.size();i++)
        {
            Eigen::Vector4f centroid_i=centroid[i];
            pcl::PointXYZ searchPoint;
            
            searchPoint.x = centroid_i[0];
            searchPoint.y = centroid_i[1];
            searchPoint.z = centroid_i[2];
            
            float overlap_ratio=0;
            float overlap_within_new_cluster;
            std::vector<int> link_index_i;
            std::vector<float> overlap_ratio_i;
            for(int j=0; j<target_clusters.size(); j++)
            {
                float radius = 0.05;
                cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(*target_cloud, target_clusters[j], *cloud_xyz);
                kdtree.setInputCloud(cloud_xyz);
                
                kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
                overlap_ratio = (float)pointIdxRadiusSearch.size()/input_clusters[i].indices.size();
                overlap_within_new_cluster = (float)pointIdxRadiusSearch.size()/target_clusters[j].indices.size();
                if( overlap_ratio > 0.2f)
                {
                    link_index_i.push_back(j);
                    overlap_ratio_i.push_back(overlap_ratio);
                }
                
                if(VERBOSE)
                {
                    std::cout << "pre_cluster[" << i << "] = " << input_clusters[i].indices.size() << " | ";
                    std::cout << "cur_cluster[" << j << "] = " << target_clusters[j].indices.size() << " | ";
                    std::cout << "radius search got " << pointIdxRadiusSearch.size() << " points | ";
                    std::cout << "overlap ratio [" << i << "," << j <<"] = " << overlap_ratio << " | ";
                    std::cout << "overlap ratio with new cluster [" << i << "," << j <<"] = " << overlap_within_new_cluster << std::endl;
                }
            }
            
            if(!link_index_i.empty())
            {
                linker linker_i;
                linker_i.link_index = link_index_i;
                linker_i.overlap_ratio = overlap_ratio_i;
                
                result.push_back(linker_i);
            }
        }
        
        return result;
    }
    
    void convertCloud(pcl::PointCloud< pcl::PointXYZRGBA >::Ptr& input_cloud, const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& output_cloud, bool keep_organized)
    {
        if(keep_organized)
        {
            output_cloud->height = input_cloud -> height;
            output_cloud->width = input_cloud -> width;
            output_cloud->points.resize(input_cloud ->height * input_cloud->width);
            
             for(int i=0; i<input_cloud->points.size(); i++)
            {   
                output_cloud->points[i].x = input_cloud->points[i].x;
                output_cloud->points[i].y = input_cloud->points[i].y;
                output_cloud->points[i].z = input_cloud->points[i].z;
                output_cloud->points[i].r = input_cloud->points[i].r;
                output_cloud->points[i].g = input_cloud->points[i].g;
                output_cloud->points[i].b = input_cloud->points[i].b;
                output_cloud->points[i].rgb = input_cloud->points[i].rgb;
            }
        }else{
            pcl::copyPointCloud(*input_cloud, *output_cloud);
            
            for(int i=0; i<input_cloud->points.size(); i++)
            {   
                output_cloud->points[i].r = input_cloud->points[i].r;
                output_cloud->points[i].g = input_cloud->points[i].g;
                output_cloud->points[i].b = input_cloud->points[i].b;
                output_cloud->points[i].rgb = input_cloud->points[i].rgb;
            }
        }
    }

    
    void convertCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& input_cloud, const pcl::PointIndices& indices, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud, bool keep_organized)
    { 
        if(keep_organized)
        {
            output_cloud->height = input_cloud -> height;
            output_cloud->width = input_cloud -> width;
            output_cloud->points.resize(input_cloud ->height * input_cloud->width);
            
            for(int i=0; i<indices.indices.size(); i++)
            {   
                output_cloud->points[indices.indices[i]].x = input_cloud->points[indices.indices[i]].x;
                output_cloud->points[indices.indices[i]].y = input_cloud->points[indices.indices[i]].y;
                output_cloud->points[indices.indices[i]].z = input_cloud->points[indices.indices[i]].z;
                
                output_cloud->points[indices.indices[i]].r = input_cloud->points[indices.indices[i]].r;
                output_cloud->points[indices.indices[i]].g = input_cloud->points[indices.indices[i]].g;
                output_cloud->points[indices.indices[i]].b = input_cloud->points[indices.indices[i]].b;
                output_cloud->points[indices.indices[i]].rgb = input_cloud->points[indices.indices[i]].rgb;
            }
        }else{
            int oldH = output_cloud ->height;
            output_cloud->width=1;
            output_cloud ->height = output_cloud->height + indices.indices.size();
            output_cloud->points.resize( output_cloud->points.size() + indices.indices.size());
            
            for(int i=0; i<indices.indices.size(); i++)
            {   
                output_cloud->points[oldH + i].x = input_cloud->points[indices.indices[i]].x;
                output_cloud->points[oldH + i].y = input_cloud->points[indices.indices[i]].y;
                output_cloud->points[oldH + i].z = input_cloud->points[indices.indices[i]].z;
    
                output_cloud->points[oldH + i].r = input_cloud->points[indices.indices[i]].r;
                output_cloud->points[oldH + i].g = input_cloud->points[indices.indices[i]].g;
                output_cloud->points[oldH + i].b = input_cloud->points[indices.indices[i]].b;
                output_cloud->points[oldH + i].rgb = input_cloud->points[indices.indices[i]].rgb;
            }
        }
        
    }
    
    void convertCloud(pcl::PointCloud< pcl::PointXYZRGBA >::Ptr& input_cloud, const pcl::PointIndices& indices, const pcl::PointCloud< pcl::PointXYZ >::Ptr& output_cloud, bool keep_organized)
    {
        if(keep_organized)
        {
            output_cloud->height = input_cloud -> height;
            output_cloud->width = input_cloud -> width;
            output_cloud->points.resize(input_cloud ->height * input_cloud->width);
            
            for(int i=0; i<indices.indices.size(); i++)
            {   
                output_cloud->points[indices.indices[i]].x = input_cloud->points[indices.indices[i]].x;
                output_cloud->points[indices.indices[i]].y = input_cloud->points[indices.indices[i]].y;
                output_cloud->points[indices.indices[i]].z = input_cloud->points[indices.indices[i]].z;
                
            }
        }else{
            int oldH = output_cloud ->height;
            output_cloud->width=1;
            output_cloud ->height = output_cloud->height + indices.indices.size();
            output_cloud->points.resize( output_cloud->points.size() + indices.indices.size());
            
            for(int i=0; i<indices.indices.size(); i++)
            {   
                output_cloud->points[oldH + i].x = input_cloud->points[indices.indices[i]].x;
                output_cloud->points[oldH + i].y = input_cloud->points[indices.indices[i]].y;
                output_cloud->points[oldH + i].z = input_cloud->points[indices.indices[i]].z;
    
            }
        }
    }

    
    void convertCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& input_cloud, const std::vector<pcl::PointIndices>& indices, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& output_cloud)
    { 
        output_cloud->width=1;
        for(int j=0; j<indices.size(); j++)
        {
            int oldH = output_cloud ->height;
            output_cloud ->height = output_cloud->height + indices[j].indices.size();
            output_cloud->points.resize( output_cloud->points.size() + indices[j].indices.size());
            
            for(int i=0; i<indices[j].indices.size(); i++)
            {   
                output_cloud->points[oldH + i].x = input_cloud->points[indices[j].indices[i]].x;
                output_cloud->points[oldH + i].y = input_cloud->points[indices[j].indices[i]].y;
                output_cloud->points[oldH + i].z = input_cloud->points[indices[j].indices[i]].z;

                output_cloud->points[oldH + i].r = input_cloud->points[indices[j].indices[i]].r;
                output_cloud->points[oldH + i].g = input_cloud->points[indices[j].indices[i]].g;
                output_cloud->points[oldH + i].b = input_cloud->points[indices[j].indices[i]].b;
                output_cloud->points[oldH + i].rgb = input_cloud->points[indices[j].indices[i]].rgb;
            }
        }
    }
    
    void convertCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& input_cloud, const pcl::PointIndices& indices, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& output_cloud)
    { 
        output_cloud->width=1;
        
        int oldH = output_cloud ->height;
        output_cloud ->height = output_cloud->height + indices.indices.size();
        output_cloud->points.resize( output_cloud->points.size() + indices.indices.size());
        
        for(int i=0; i<indices.indices.size(); i++)
        {   
            output_cloud->points[oldH + i].x = input_cloud->points[indices.indices[i]].x;
            output_cloud->points[oldH + i].y = input_cloud->points[indices.indices[i]].y;
            output_cloud->points[oldH + i].z = input_cloud->points[indices.indices[i]].z;

            output_cloud->points[oldH + i].r = input_cloud->points[indices.indices[i]].r;
            output_cloud->points[oldH + i].g = input_cloud->points[indices.indices[i]].g;
            output_cloud->points[oldH + i].b = input_cloud->points[indices.indices[i]].b;
            output_cloud->points[oldH + i].rgb = input_cloud->points[indices.indices[i]].rgb;
        }
        
    }
    
    float distance(pcl::PointXYZRGB p1, pcl::PointXYZRGBA p2)
    {
        float distance =  sqrt(pow((int)p1.r-(int)p2.r,2)
                              +pow((int)p1.g-(int)p2.g,2)
                              +pow((int)p1.b-(int)p2.b,2));
        
        return distance;
    }

    float distance(pcl::PointXYZRGBA p1, pcl::PointXYZ p2)
    {
        float distance =  sqrt(pow((float)p1.x-(float)p2.x,2)
                                +pow((float)p1.y-(float)p2.y,2)
                                +pow((float)p1.z-(float)p2.z,2));
        
        return distance;
    }
    
    float distance(pcl::PointXYZ p1, pcl::PointXYZ p2)
    {
        float distance =  sqrt(pow((float)p1.x-(float)p2.x,2)
                              +pow((float)p1.y-(float)p2.y,2)
                              +pow((float)p1.z-(float)p2.z,2));
        
        return distance;
    }
    
    float normalizedDistance(pcl::PointXYZRGB p1, pcl::PointXYZRGBA p2)
    {
        float normalize_search = sqrt(pow((int)p1.r,2)+pow((int)p1.g,2)+pow((int)p1.b,2));
        float normalize_nearest = sqrt(pow((int)p2.r,2)+pow((int)p2.g,2)+pow((int)p2.b,2));
        
        float color_distance = sqrt(pow(float((float)p1.r/normalize_search)-float((float)p2.r/normalize_nearest),2)
                                   +pow(float((float)p1.g/normalize_search)-float((float)p2.g/normalize_nearest),2)
                                   +pow(float((float)p1.b/normalize_search)-float((float)p2.b/normalize_nearest),2));
        
        return color_distance;
    }

    std::vector<int> findHand(const CloudPtr& cloud, const std::vector< pcl::PointIndices >& clusters, pcl::PointIndices f_indices)
    {
        std::vector<int> hand_arm;
        int hand=-1;
        int ref_index = f_indices.indices[0];
        for(int i=0; i<clusters.size(); i++)
        {
            for(int j=0; j<clusters[i].indices.size(); j++)
            {
                if(ref_index == clusters[i].indices[j])
                {
                    hand = i;
                    hand_arm.push_back(i);
                    std::cout << "hand index = " << i << std::endl;
                    break;
                }
            }
            if(hand>=0) break;
        }
        
        CloudPtr hand_cloud(new Cloud);
        if(hand>=0)
            pcl::copyPointCloud(*cloud, clusters[hand], *hand_cloud);
        else
            pcl::copyPointCloud(*cloud, f_indices, *hand_cloud);
        
        TableObject::touchDetector touchDetector(0.02);
        for(int i=0; i<clusters.size(); i++)
        {
            if(i!=hand)
            {
                CloudPtr object(new Cloud);
                pcl::copyPointCloud(*cloud, clusters[i], *object);
                if(touchDetector.detect(hand_cloud, object))
                {
                    hand_arm.push_back(i);
                    std::cout << "arm index = " << i << std::endl;
                }
            }
            
        }
        
        std::sort(hand_arm.begin(), hand_arm.end());
        return hand_arm;
        
    }

    
}
