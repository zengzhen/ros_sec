/**
 * \file        view2D.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "ros_sec/Visualizer/view2D.h"
#include <pcl/filters/extract_indices.h>
#include <iostream>

namespace TableObject{
    
    void view2D::viewImage(CloudPtr cloud, std::string imageName)
    {
        if(cloud->isOrganized())
        {
            rgbd_vision::convert_rgbd_to_image(cloud, _image); 
            cv::imshow(imageName,_image);
            cv::waitKey(-1);
        }else
        {
            std::cerr<<"input point cloud is not organized, cannot be visualized properly\n";
            exit(1);
        }
    }
    
    void view2D::viewImage(pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud, std::string imageName)
    {
        if(cloud->isOrganized())
        {
            rgbd_vision::convert_rgbd_to_image(cloud, _image); 
            cv::imshow(imageName,_image);
            cv::waitKey(-1);
        }else
        {
            std::cerr<<"input point cloud is not organized, cannot be visualized properly\n";
            exit(1);
        }
    }


    void view2D::writeImage(const std::string& fileName)
    {
        cv::imwrite(fileName, _image);
    }
    
    void view2D::writeRGBD(CloudPtr cloud, std::string imageNameBase)
    {
        if(cloud->isOrganized())
        {
            rgbd_vision::convert_pcd_to_rgbd(cloud, _image, _depth);
        }else
        {
            std::cerr<<"input point cloud is not organized, cannot be visualized properly\n";
            exit(1);
        }
        
        std::string imageName = imageNameBase + "_color.png";
        std::string depthName = imageNameBase + "_depth.png";
        
        cv::imwrite(imageName, _image);
        cv::imwrite(depthName, _depth);
    }

}