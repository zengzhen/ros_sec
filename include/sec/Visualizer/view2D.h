/**
 * \file        view2D.h
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       view 3d point cloud in 2d image
 */

#ifndef VIEW2D_H
#define VIEW2D_H

#include <pcl/point_cloud.h>
#include "Kinect/src/image/convert_rgbd_to_image.h"

#include "typeDef.h"

namespace TableObject{
    
    class view2D {
    public:
        /** \brief view 2D image
         */
        void viewImage(CloudPtr cloud, std::string imageName);
        void viewImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string imageName);
        void writeImage(const std::string& fileName);
        void writeRGBD(CloudPtr cloud, std::string imageNameBase);
    
    private:
    cv::Mat _image;
    cv::Mat _depth;
    };
    
}

#endif  // VIEW2D_H