/**
 * \file        trackRigid.h
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       track motions induced by rigid transformations
 */

#ifndef TRACKRIGID_H
#define TRACKRIGID_H

#include "ros_sec/typeDef.h"
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace TableObject{
    
    class trackRigid {
    public:
        /** \brief constructor
         */
        trackRigid();
        
        /** \brief initiazlier
        *  \param[in] cloud initial cloud (1st frame)
        *  \param[in] clusters initial clusteres (1st frame)
        */
        void init(CloudPtr cloud, const std::vector<pcl::PointIndices>& clusters);
        
        /** Tracking using whole newly arrived cloud (without segmented clusters)
         *  \param[in] inputCloud newly arrived point cloud frame
         *  \param[out] tracked_clusters indicies of tracked rigid transformed clusters(objects)
         */
        void track(CloudPtr inputCloud, std::vector<pcl::PointIndices>& tracked_clusters);
        
        /** link previous frame cloud to current frame cloud
         *  \param[in] inputCloud current frame cloud
         *  \param[in] inputClusters current frame cloud cluster
         */
        void linking(CloudPtr inputCloud, const std::vector<pcl::PointIndices>& inputClusters);
        
        /** Provide linking relation between previous and current frame cloud
         *  \param[in] link_to provided linking relation
         *  \param[in] link_clusters provided clusters for current frame cloud
         */
        void setLinkTo(const linkerList& link_to, const std::vector<pcl::PointIndices>& link_clusters);
        
        void getTransformedCloud(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& transformed_cloud);
        
        void getTestClusters(std::vector<pcl::PointIndices>& clusters);
        
        void viewTranformedCloud(pcl::visualization::PCLVisualizer& viewer, int cluster_id);
        
        void viewTrackedCloud(pcl::visualization::PCLVisualizer& viewer, int cluster_id, int r, int g, int b);
        
    private:
        CloudPtr _cloud;
        std::vector<pcl::PointIndices> _clusters;
        linkerList _link_to;
        std::vector<pcl::PointIndices> _link_clusters;
         
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> _transformed_cloud; //transformed previous frame cloud to align with current frame cloud in format PointXYZ
        
    };
    
}

#endif  // TRACKRIGID_H