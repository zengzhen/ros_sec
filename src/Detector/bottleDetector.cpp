/**
 * \file bottleDetector.cpp
 * \author Zhen Zeng (zengzhen@umich.edu)
 * \brief Cylinder detector to fit cylinder-alike bottle, and build bottle features
 */

#include "ros_sec/Detector/bottleDetector.h"

namespace TableObject{
    
    bottleDetector::bottleDetector()
    :_coefficients(new pcl::ModelCoefficients),
     _normals(new pcl::PointCloud<pcl::Normal>),
     _inliers_cylinder(new pcl::PointIndices),
     _inputCloud(new Cloud),
     _cylinderCloud(new Cloud)
    {}
    
    void bottleDetector::setInputCloud(CloudPtr inputCloud)
    {
        if(inputCloud->isOrganized())
        {
            std::cerr << "bottleDetector expects unorganized cloud\n";
            exit(1);
        }else{
            _inputCloud = inputCloud;
        }
    }
    
    void bottleDetector::fit()
    { 
        pcl::NormalEstimationOMP<RefPointType, pcl::Normal> nest;
        pcl::search::KdTree<RefPointType>::Ptr tree (new pcl::search::KdTree<RefPointType> ());
        pcl::SACSegmentationFromNormals<RefPointType, pcl::Normal> seg; 
        
        // Estimate point normals
        nest.setSearchMethod (tree);
        nest.setInputCloud (_inputCloud);
        nest.setKSearch (50);
        nest.compute (*_normals);

        // Fit cylinder model
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_CYLINDER);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight (0.1);
        seg.setMaxIterations (10000);
        seg.setDistanceThreshold (0.05);
        seg.setRadiusLimits (0, 0.04);
        seg.setInputCloud (_inputCloud);
        seg.setInputNormals (_normals);

        // Obtain the cylinder inliers and coefficients
        seg.segment (*_inliers_cylinder, *_coefficients);
        std::cout << "Cylinder coefficients: " << *_coefficients << std::endl;
//         computeCylinderCloud();

        computeFeatures();
    }
    
    void bottleDetector::getCoefficients(pcl::ModelCoefficients& coefficients)
    {
        coefficients = *_coefficients;
    }
    
    float bottleDetector::getHeight()
    {
        return _height;
    }
    
    float bottleDetector::getRadius()
    {
        return _radius;
    }
    
    RefPointType bottleDetector::getCenter()
    {
        return _center;
    }
    
    Eigen::Vector3f bottleDetector::getOrientation()
    {
        return _orientation;
    }
    
    void bottleDetector::getTransformation(Eigen::Affine3f& tranform)
    {
        tranform = _transform;
    }
    
    void bottleDetector::drawCylinder(pcl::visualization::PCLVisualizer& viewer)
    {
        viewer.addCylinder(*_coefficients, "bottle_cylinder");
    }
    
    void bottleDetector::drawCenter(pcl::visualization::PCLVisualizer& viewer)
    {
        viewer.addSphere<RefPointType>(_center, 0.05, "bottle_center");
    }
    
    void bottleDetector::drawOrientation(pcl::visualization::PCLVisualizer& viewer)
    {
        pcl::PointXYZ bottle_arrow(_center.x+_orientation[0]*0.3, _center.y+_orientation[1]*0.3, _center.z+_orientation[2]*0.3);
        viewer.removeShape("bottle_arrow");
        viewer.addArrow(bottle_arrow, _center, 1, 0, 0, 0,"bottle_arrow");
    }
    
    void bottleDetector::drawNormals(pcl::visualization::PCLVisualizer& viewer)
    {
        viewer.addPointCloudNormals<RefPointType,pcl::Normal>(_inputCloud, _normals);
    }
    
    void bottleDetector::computeCylinderCloud()
    {
        pcl::ExtractIndices<RefPointType> extract;
        
        extract.setInputCloud (_inputCloud);
        extract.setIndices (_inliers_cylinder);
        extract.setNegative (false);
        extract.filter (*_cylinderCloud);
        if (_cylinderCloud->points.empty ()) 
            std::cerr << "Can't find the cylindrical component." << std::endl;
        else
        {
            std::cout << "PointCloud representing the cylindrical component: " << _cylinderCloud->points.size () << " data points." << std::endl;
        }
    }
    
    void bottleDetector::computeFeatures()
    {
        _radius = _coefficients->values[6];
        
        Eigen::Vector3f p( _coefficients->values[0], _coefficients->values[1], _coefficients->values[2] ); // position
        _orientation[0] = _coefficients->values[3]; 
        _orientation[1] = _coefficients->values[4];
        _orientation[2] = _coefficients->values[5];
        if(_orientation[1]>0) _orientation=-_orientation;

        // get a vector that is orthogonal to _orientation ( yc = _orientation x [1,0,0]' ) 
        Eigen::Vector3f yc( 0, _orientation[2], -_orientation[1] ); 
        yc.normalize(); 
        // get a transform that rotates _orientation into z and moves cloud into origin. 
        pcl::getTransformationFromTwoUnitVectorsAndOrigin(yc, _orientation, p, _transform); 
        
        // get cylinder height
        CloudPtr cluster_bottle_transformed (new Cloud); 
        pcl::transformPointCloud( *_inputCloud, *cluster_bottle_transformed, _transform );
        RefPointType min_pt, max_pt;
        pcl::getMinMax3D<RefPointType>(*cluster_bottle_transformed, min_pt, max_pt);
        _height = max_pt.z-min_pt.z;
        
        // get cylinder center
        pcl::PointXYZ center(0,0,(float)((max_pt.z+min_pt.z)/2));
        center = pcl::transformPoint<pcl::PointXYZ>(center, _transform.inverse());
        _center.x = center.x; _center.y = center.y; _center.z = center.z;
        
        // get transform to bottle coordinate (_center being the origin, _orientation being the z axis)
        Eigen::Vector3f center_vector(center.x, center.y, center.z);
        std::cout << center << std::endl;
        pcl::getTransformationFromTwoUnitVectorsAndOrigin(yc, _orientation, center_vector, _transform); 
        
        // get _inputCloud mean color
        pcl::CentroidPoint<RefPointType> color_points;
        for(int j=0; j<_inputCloud->points.size();j++)
        {
            color_points.add(_inputCloud->points[j]);
        }
        RefPointType mean_color;
        color_points.get(mean_color);
        _center.r = mean_color.r; _center.g = mean_color.g; _center.b = mean_color.b;
    }
}
