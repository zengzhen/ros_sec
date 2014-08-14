/**
 * \file        pcd_cloud.h
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       basic processing for point cloud data (loading, filtering, extraction)
 * \brief       and dominant plane segmentation, region growing based on distance (or/and) color
 */

#ifndef PCD_CLOUD_H
#define PCD_CLOUD_H

#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>


#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/region_growing_rgb.h>

#include "typeDef.h"

namespace TableObject{
    
    class pcdCloud {
    public:
        
        /** \brief empty Constructor.
        */
        pcdCloud();
        
        /** \brief Constructor.
        *   \param[in] pcd_file path to .pcd file
        */
        pcdCloud(const std::string & pcd_file);
        
        /** \brief Constructor.
        *   \param[in] cloud existing point cloud pointer (note: not copy, but point to the exact point cloud)
        */
        pcdCloud(CloudConstPtr cloud);
        
        /** \brief set cloud to pcd file
         *  \param[in] pcd_file path to .pcd file
         */
        void resetCloud(const std::string & pcd_file);
        
        
        /** \brief set cloud
         *  \param[in] cloud set _cloud
         */
        void resetCloud(CloudConstPtr cloud);
        
        /** \brief set indices for valid indices used by any operations
         *  \param[in] reservedIndices set _reservedIndices
         */
        void setIndices(pcl::IndicesPtr reservedIndices);
        
        /** \brief set indices for valid indices used by any operations
         *  \param[in] reservedIndices set _reservedIndices
         */
        void setIndices (const pcl::PointIndices indices);
        
        /** Filter out points too far away
        */
        void filterValid();
        
        /** Filter out noisy points statistically
         */
        void filterNoise();
        
        /** Extract points to a new organized point cloud
         * \param[out] output extracted point cloud
         * \param[in] inliers set indices of inliers of output point cloud
         * \param[in] invert (default)false: extract inliers; true: extract outliers
         */
        void extract(CloudPtr output,  pcl::IndicesPtr inliers, bool invert);
        
        /** Extract points to a new unorganized point cloud, optionally output removed indices
         * \param[out] output extracted point cloud
         * \param[in] inliers set indices of inliers of output point cloud
         * \param[in] invert (default)false: extract inliers; true: extract outliers
         * \param[in] outliers (optional)get removed indices
         */
        void extract(CloudPtr output,  pcl::IndicesPtr inliers, pcl::IndicesPtr outliers, bool invert);
        
        /** fit a plane to the point cloud
         * NOTE: segmentation needs unorganized point cloud to work well (not sure if it is still true)
         * \param[in] threshold distance to model threshold
         */
        void findPlane(double threshold);
        
        /** extract in-plane and out-plane point clouds
         * \param[in] inPlaneCloud   extracted in-plane point cloud 
         * \param[in] outPlaneCloud  extracted out-plane point cloud
         */
        void extractPlane(CloudPtr inPlaneCloud, CloudPtr outPlaneCloud);
        
        /** \brief region growing for clustering points based on 3D distances
         * \param[in] colorOn (default)false if region growing based on just 3D idstances; true if region growing also based on color;
         */
        void regionGrow(const bool& colorOn);
        
        /** \brief Accessesor.
         */
        CloudPtr getCloud();
        void getRemovedIndices(pcl::IndicesPtr removedIndices);
        void getReservedIndices(pcl::IndicesPtr reservedIndices);
        void getPlaneCoefficients(pcl::ModelCoefficients& coefficients);
        void getPlaneInliers(pcl::IndicesPtr planeInliers);
        void getClusters(std::vector <pcl::PointIndices>& clusters);
        void getThresholdedClusters(std::vector <pcl::PointIndices>& clusters, int threshold);
        void getMaxCluster(pcl::PointIndices::Ptr plane_indices, CloudPtr maxCluster);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColoredCloud();
        
    private:
        /** \brief update removedIndices everytime reservedIndices change
         */
        void updateRemovedIndices();
        /** \brief update reservedIndices everytime removedIndices change
         */
        void updateReservedIndices();
        
        CloudPtr _cloud;
        /** list of removed indices in the coordinates of the organized point cloud: _cloud
         * concatenate all indices of removed entries since the initialization/construction of _cloud
         */
        pcl::IndicesPtr _removedIndices;
        pcl::IndicesPtr _reservedIndices;
        pcl::ExtractIndices<RefPointType> _extractor;
        pcl::IndicesPtr _planeInliers;
        pcl::ModelCoefficients::Ptr _coefficients;
        std::vector <pcl::PointIndices> _clusters;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _colored_cloud;
    };
    
}

#endif  // PCD_CLOUD_H