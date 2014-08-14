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
#include "Tracker/track3D.h"

namespace TableObject{
    
    class view3D {
    public:
        /** \brief view 3D particles
         */
//         static bool drawParticles (pcl::visualization::PCLVisualizer& viz, TableObject::track3D& tracker);
        
        /** \brief view tracked 3D object
         */
        static CloudPtr drawResult (pcl::visualization::PCLVisualizer& viz, TableObject::track3D& tracker);
        
        /** \brief view object clusters with diff random colors*/
        static void drawClusters(pcl::visualization::PCLVisualizer& viz, CloudPtr cloud, std::vector<pcl::PointIndices> clusters);
        
        /** \brief add object index notion for each cloud*/
        static void drawText(pcl::visualization::PCLVisualizer& viz, CloudPtr cloud, std::vector<pcl::PointIndices> clusters);
    };
    
}

#endif  // VIEW3D_H