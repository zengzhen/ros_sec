/**
 * \file        util.h
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       utilities
 */

#ifndef UTIL_H
#define UTIL_H

#include "ros_sec/typeDef.h"
#include <Eigen/StdVector>

namespace TableObject{
    
    /** \brief compute vector of centroids for cloud clusters
    *  \param[in] cloud cloud data
    *  \param[in] clusters vector of cluster point indices
    *  \param[out centroid computed centroids
    */
    void computeObjCentroid(CloudPtr cloud, std::vector<pcl::PointIndices> clusters, std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> >& centroid);

	/** \brief compute vector of centroids for cloud clusters
    *  \param[in] cloud cloud data
    *  \param[out centroid computed centroids
    */
    pcl::PointXYZ computeObjCentroid(CloudPtr cloud);
    
    /** \brief link clusters of input_cloud to clusters of target_cloud based on overlapping ratio
    *  \param[in] input_cloud input cloud data
    *  \param[in] input_clusters cluster indices of input cloud
    *  \param[in] target_cloud target cloud data
    *  \param[in] target_clusters cluster indices of target cloud
    *  \return computed linked index correspond to each of the input clusters, and overlap ratio
    */
    linkerList linking(CloudPtr input_cloud, const std::vector<pcl::PointIndices>& input_clusters, CloudPtr target_cloud, const std::vector<pcl::PointIndices>& target_clusters);
    
    /** \brief convert point cloud with PointXYZRGBA to point cloud with PointXYZRGB (preserve x,y,z and also r,g,b infomation!!!
     *  \brief NOTE: pcl::copyPointCloud does not preserve r g b infomation!!!
    *  \param[in] input_cloud source cloud data
    *  \param[out] output_cloud destination cloud data
    */
    void convertCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& input_cloud, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud, bool keep_organized);
    
    
    /** \brief convert point cloud with PointXYZRGBA to point cloud with PointXYZRGB (preserve x,y,z and also r,g,b infomation!!!
     *  \brief NOTE: pcl::copyPointCloud does not preserve r g b infomation!!!
    *  \param[in] input_cloud source cloud data (size: height * width)
    *  \param[in] indices indices of points to be copied
    *  \param[in] keep_organized true: size(output_cloud)=size(intput_cloud); false: size(output_cloud)=size(indices)
    *  \param[out] output_cloud destination cloud data (size: heigth * width, same as the input_cloud, keep organized)
    */
    void convertCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& input_cloud, const pcl::PointIndices& indices, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud, bool keep_organized);
    
    /** \brief convert point cloud with PointXYZRGBA to point cloud with PointXYZ
    *  \param[in] input_cloud source cloud data (size: height * width)
    *  \param[in] indices indices of points to be copied
    *  \param[in] keep_organized true: size(output_cloud)=size(intput_cloud); false: size(output_cloud)=size(indices)
    *  \param[out] output_cloud destination cloud data (size: heigth * width, same as the input_cloud, keep organized)
    */
    void convertCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& input_cloud, const pcl::PointIndices& indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud, bool keep_organized);
    
    
    /** \brief extract input_cloud point indices as given to unorganzied output_cloud
    *  \param[in] input_cloud source cloud data (size: height * width)
    *  \param[in] indices indices of points to be copied
    *  \param[out] output_cloud destination cloud data (size: cluster_indices size * 1)
    */
    void convertCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& input_cloud, const std::vector<pcl::PointIndices>& indices, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& output_cloud);
    
    /** \brief extract input_cloud point indices as given to unorganzied output_cloud
    *  \param[in] input_cloud source cloud data (size: height * width)
    *  \param[in] indices indices of points to be copied (not vectors)
    *  \param[out] output_cloud destination cloud data (size: cluster_indices size * 1)
    */
    void convertCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& input_cloud, const pcl::PointIndices& indices, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& output_cloud);
    
    float distance(pcl::PointXYZRGB p1, pcl::PointXYZRGBA p2);

    float distance(pcl::PointXYZRGBA p1, pcl::PointXYZ p2);
    
    float distance(pcl::PointXYZ p1, pcl::PointXYZ p2);

    float normalizedDistance(pcl::PointXYZRGB p1, pcl::PointXYZRGBA p2);
    
    /** \brief find which cluster(s) is the hand-arm (contain the finger indices or touching the cluster that constains the finger indices)
    *  \param[in] cloud object cloud
    *  \param[in] clusters cluster indices
    *  \param[in] f_indices finger indices
    */
    std::vector<int> findHand(const CloudPtr& cloud, const std::vector<pcl::PointIndices>& clusters,  pcl::PointIndices f_indices);
}

#endif  // UTIL_H
