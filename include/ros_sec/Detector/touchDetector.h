/**
 * \file        touchDetector.h
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       touch detector between objects
 */

#ifndef TOUCHDETECTOR_H
#define TOUCHDETECTOR_H

#include "ros_sec/typeDef.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "ros_sec/TableObjectSegmentation/pcd_cloud.h"

namespace TableObject{
    
    class touchDetector {
        
        public:
        /** \brief Constructor
        */
        touchDetector(double threshold);
        
        /** \brief detect if two clouds are touching each other by examine the distance between the neearset point pair
        *   \param[in] cloud_1 first point cloud
        *   \param[in] cloud_2 first point cloud
        */
        bool detect(const CloudPtr& cloud_1, const CloudPtr& cloud_2);

        /** \brief detect if points within a cloud are touching the tabletop by examine the distance between the neearset point pair
        *   \param[in] cloud first point cloud
        *   \param[in] coefficients for tabletop plane
        */
        bool detectTableTouch(const CloudPtr& cloud_1, pcl::ModelCoefficients);
        
        /** \brief display the relation just detected, based on string, at location (100,y)
         *  \param[in] visualizer
         */
        void showTouch(pcl::visualization::PCLVisualizer& viewer, std::string pair, int x, int y);
        
    private:
        double _threshold;
        bool _touch;
    };
}

#endif  // TOUCHDETECTOR_H
