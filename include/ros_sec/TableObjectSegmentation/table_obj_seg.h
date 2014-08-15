/**
 * \file        table_obj_seg.h
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       segment tabletop objects from given point cloud or .pcd file
 */

#ifndef TABLE_OBJ_SEG_H
#define TABLE_OBJ_SEG_H

#include "ros_sec/TableObjectSegmentation/pcd_cloud.h"
#include "ros_sec/Visualizer/view2D.h"

namespace TableObject
{
    class Segmentation {
    public:
        /** \brief empty Constructor.
        */
        Segmentation();
        
        /** \brief Constructor.
        *   \param[in] pcd_file path to .pcd file
        */
        Segmentation(const std::string & pcd_file);
        
        /** \brief Constructor.
        *   \param[in] cloud existing point cloud pointer (note: not copy, but point to the exact point cloud)
        */
        Segmentation(CloudConstPtr cloud);
        
        /** \brief set scene cloud to pcd file */
        void resetCloud(const std::string & pcd_file, bool downsample);
        
        /** \brief set scene cloud */
        void resetCloud(CloudConstPtr cloud);
        
        /** \brief set threshold for size of extracted object */
        void setThreshold(int threshold);
        
        /** \brief segment object from tabletop 
         *  \param[in] view2D (default)true if display intermediate results with 2D images; false else.
         */
        void seg(bool view2D=true);
        
        /** \brief segment object from tabletop 
         *  \param[in] CloudPtr provide cloud_hull (extracted at first frame)
         *  \param[in] view2D (default)true if display intermediate results with 2D images; false else.
         */
        void seg(CloudPtr cloud_hull, bool view2D=true);
        
        /** \brief prune point clusters (objects) that are flat and has centroid close to the tabletop
         */
        void prune();
        
         /** \brief get pcdCloud _sceneCloud 
         *  \param[in] sceneCloud output sceneCloud
         */
        void getsceneCloud(pcdCloud & sceneCloud);
        
        /** \brief get segmented object clusters 
         *  \param[in] cloud_objects output extracted cloud_objects
         *  \param[in] clusters output indices for each cluster
         */
        void getObjects(CloudPtr cloud_objects, std::vector<pcl::PointIndices>& clusters);
        
        /** \brief get in plane cloud
         *  \param[in] inPlaneCloud output dominant plane cloud
         */
        void getInPlaneCloud(CloudPtr inPlaneCloud);
        
        /** \brief get out of plane cloud
         *  \param[in] outPlaneCloud output points other than dominant plane cloud
         */
        void getOutPlaneCloud(CloudPtr outPlaneCloud);
        
        /** \brief get cloud hull for table top
         *  \param[in] outPlaneCloud output points other than dominant plane cloud
         */
        void getCloudHull(CloudPtr cloudHull);
        
        /** \brief get table top cloud
         *  \param[in] tableTopCloud output table top point cloud
         */
        void getTableTopCloud(CloudPtr tableTopCloud);

        void getPlaneCoefficients(pcl::ModelCoefficients& coefficients);
        
        /** \brief record target object */
        void recordTarget(int cluster_id);
        
        void computeObjCentroid(std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> >& centroid);
        
    private:
        pcdCloud _sceneCloud;
        int _threshold;
        TableObject::view2D _viewer2D;
        
        CloudPtr _inPlaneCloud;
        CloudPtr _outPlaneCloud;
        CloudPtr _cloud_hull;
        CloudPtr _tableTopCloud;
        CloudPtr _cloud_objects;
        std::vector< pcl::PointIndices > _clusters;
        pcl::ModelCoefficients _coefficients;
    };
}

#endif  // TABLE_OBJ_SEG_H
