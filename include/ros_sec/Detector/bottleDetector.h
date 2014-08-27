/**
 * \file        bottleDetector.h
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 * \brief       Cylinder detector to fit cylinder-alike bottle, and build bottle features
 */

#ifndef BOTTLEDETECTOR_H
#define BOTTLEDETECTOR_H

#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ros_sec/typeDef.h"

namespace TableObject{
    class bottleDetector {
    public:
        /**
         * \brief default constructor
         */
        bottleDetector();
        
        /**
         * \brief set input cloud to fit cylinder model
         * \param[in] inputCloud
         */
        void setInputCloud(CloudPtr inputCloud);
        
        /**
         * \brief fit cylinder model to _inputCloud and get bottle features
         */
        void fit();
        
        /**
         * \brief get fitted cylinder coefficients
         * \param[out] coefficients cylinder coefficients
         */
        void getCoefficients(pcl::ModelCoefficients& coefficients);
        
        /**
         * \brief get bottle height
         */
        float getHeight();
        
        /**
         * \brief get bottle radius
         */
        float getRadius();
        
        /*! \brief get bottle center 
         *     
         *  (x,y,z): cylinder model center, instead of inputCloud center
         *  (r,g,b): mean color of inputCloud 
         */
        RefPointType getCenter();
        
        /**
         * \brief get bottle orientation
         */
        Eigen::Vector3f getOrientation();
        
        /**
         * \brief get the transformation (from world coordinate to bottle coordiante)
         */
        void getTransformation(Eigen::Affine3f& transform);
        
        /**
         * \brief show fitted cylinder model (NOTE: cylinder does not have limited height)
         */
        void drawCylinder(pcl::visualization::PCLVisualizer& viewer);
        
        /**
         * \brief show fitted cylinder center
         */
        void drawCenter(pcl::visualization::PCLVisualizer& viewer);
        
        /**
         * \brief show fitted cylinder orientation (in the form of arrow)
         */
        void drawOrientation(pcl::visualization::PCLVisualizer& viewer);
        
        /**
         * \brief show bottle cloud normals
         */
        void drawNormals(pcl::visualization::PCLVisualizer& viewer);
        
    private:
        
        /**
         * \brief compute cylinder inliers in _inputCloud
         */
        void computeCylinderCloud();
        
        /**
         * \brief compute bottle features
         */
        void computeFeatures();
        
        float _height, _radius;
        Eigen::Vector3f _orientation;
        RefPointType _center;
        
        // coefficients_cylinder: center.x center.y center.z Axis.x Axis.y Axis.z Radius.
        // center is some point along the inifinite long axis, not related to center at all
        // Axis.x Axis.y Axis.z -> unit orientation vector
        pcl::ModelCoefficients::Ptr _coefficients; 
        pcl::PointCloud<pcl::Normal>::Ptr _normals;
        pcl::PointIndices::Ptr _inliers_cylinder;
        
        CloudPtr _inputCloud;
        CloudPtr _cylinderCloud;
        
        Eigen::Affine3f _transform;
    };
}

#endif //BOTTLEDETECTOR_H
