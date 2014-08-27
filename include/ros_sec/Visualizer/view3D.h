/**
 * \file        view3D.h
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       view 3d tracked object, particles, object clusters with random color
 */

#ifndef VIEW3D_H
#define VIEW3D_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "ros_sec/Tracker/track3D.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace TableObject{
    
    class view3D {
    public:       
        
        /** \brief view object clusters with diff random colors*/
        static void drawClusters(pcl::visualization::PCLVisualizer& viz, CloudPtr cloud, std::vector<pcl::PointIndices> clusters, bool showText);
        
        /** \brief add object index notion for each cloud*/
        static void drawText(pcl::visualization::PCLVisualizer& viz, CloudPtr cloud, std::vector<pcl::PointIndices> clusters);
                
        /** \brief draw arrow based on given point, orientation and transformation
         * \param[in] viz
         *  \param[in] point 
         *  \param[in] orientation
         *  \param[in] transformation
         */
        static void drawArrow(pcl::visualization::PCLVisualizer& viz, pcl::PointXYZ point, Eigen::Vector3f orientation, std::string id);
    };
    
}

#endif  // VIEW3D_H