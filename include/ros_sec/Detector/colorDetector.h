/**
 * \file        colorDetector.h
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       RGB color detection to get objects with known color (eg, marked fingers)
 */

#ifndef COLORDETECTOR_H
#define COLORDETECTOR_H

#include <pcl/visualization/pcl_visualizer.h>
#include "ros_sec/Visualizer/view2D.h"
#include "ros_sec/typeDef.h"
#include <cv.h>

namespace TableObject{
    class colorDetector {
    public:
        /** \brief empty Constructor
        */
        colorDetector(){};
        
        /** \brief Constructor
        *  \param[in] rl/rh: low/high threshold of "r" value
        *  \param[in] gl/gh: low/high threshold of "g" value
        *  \param[in] bl/bh: low/high threshold of "b" value
        */
        colorDetector(int rl, int rh, int gl, int gh, int bl, int bh);
        
        void setInputCloud(const CloudPtr& cloud, const std::vector<pcl::PointIndices>& clusters);
        
        void filter(pcl::PointIndices& fingertip_ptIdx, CloudPtr& detected_cloud);
        
        void showDetectedCloud(pcl::visualization::PCLVisualizer& viewer, std::string cloud_name);
        
    private:
        int _rl, _rh, _gl, _gh, _bl, _bh;
        CloudPtr _cloud;
        CloudPtr _detectedCloud;
        pcl::PointIndices _fingertip_ptIdx;
        std::vector<pcl::PointIndices> _clusters;
    };
    
}

#endif  // COLORDETECTOR_H
