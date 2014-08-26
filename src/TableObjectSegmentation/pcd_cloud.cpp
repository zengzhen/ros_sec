/**
 * \file        pcd_cloud.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "ros_sec/TableObjectSegmentation/pcd_cloud.h"
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <time.h>

namespace TableObject{
    pcdCloud::pcdCloud()
    :_cloud(new Cloud),
    _removedIndices(new std::vector<int>),
    _reservedIndices(new std::vector<int>),
    _planeInliers(new std::vector<int>),
    _coefficients(new pcl::ModelCoefficients){}

    
    pcdCloud::pcdCloud(const std::string & pcd_file)
    :_cloud(new Cloud),
    _removedIndices(new std::vector<int>),
    _reservedIndices(new std::vector<int>),
    _planeInliers(new std::vector<int>),
    _coefficients(new pcl::ModelCoefficients)
    {
        if (pcl::io::loadPCDFile<RefPointType> (pcd_file, *_cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file\n");
            exit(1);
        } else
        {
            std::cout << "point cloud: width = " << _cloud->width << "; height = " << _cloud->height << "; totoal = " << _cloud->width*_cloud->height << std::endl;
            //downsample the point cloud
            std::clock_t t = std::clock();
            pcl::VoxelGrid<RefPointType> sampler;
            sampler.setInputCloud(_cloud);
            sampler.setLeafSize(0.005f, 0.005f, 0.005f);
            sampler.filter(*_cloud);
            std::cout << "point cloud: width = " << _cloud->width << "; height = " << _cloud->height << "; totoal = " << _cloud->width*_cloud->height << std::endl;
            t = std::clock() - t;
        }
    }
    
    pcdCloud::pcdCloud(CloudConstPtr cloud)
    :_cloud(cloud), 
    _removedIndices(new std::vector<int>),
    _reservedIndices(new std::vector<int>),
    _planeInliers(new std::vector<int>),
    _coefficients(new pcl::ModelCoefficients)
    {
        std::cout << "point cloud: width = " << _cloud->width << "; height = " << _cloud->height << "; totoal = " << _cloud->width*_cloud->height << std::endl;
    }
    
    void pcdCloud::resetCloud(const std::string& pcd_file, bool downsample)
    {
        if (pcl::io::loadPCDFile<RefPointType> (pcd_file, *_cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file\n");
            exit(1);
        } else
        {
            std::cout << "point cloud: width = " << _cloud->width << "; height = " << _cloud->height << "; totoal = " << _cloud->width*_cloud->height << std::endl;
            if(downsample) 
            {
                //downsample the point cloud
                std::clock_t t = std::clock();
                pcl::VoxelGrid<RefPointType> sampler;
                sampler.setInputCloud(_cloud);
                sampler.setLeafSize(0.0001f, 0.0001f, 0.0001f);
                sampler.filter(*_cloud);
                
                std::cout << "point cloud: width = " << _cloud->width << "; height = " << _cloud->height << "; totoal = " << _cloud->width*_cloud->height << std::endl;
                t = std::clock() - t;
                std::printf("downsample cloud: %f seconds\n", ((float)t)/CLOCKS_PER_SEC);
            }
        }
        
        _removedIndices.reset(new std::vector<int>);
        _reservedIndices.reset(new std::vector<int>);
        _planeInliers.reset(new std::vector<int>);
        _coefficients.reset(new pcl::ModelCoefficients);
    }

    void pcdCloud::resetCloud(CloudConstPtr cloud)
    {
//         _cloud=cloud;
        _cloud.reset(new Cloud);
        pcl::copyPointCloud(*cloud, *_cloud);
        
        _removedIndices.reset(new std::vector<int>);
        _reservedIndices.reset(new std::vector<int>);
        _planeInliers.reset(new std::vector<int>);
        _coefficients.reset(new pcl::ModelCoefficients);
        
    }

    void pcdCloud::setIndices(pcl::IndicesPtr reservedIndices)
    {
        if(VERBOSE) std::cout << "setting indices...\n";
        *_reservedIndices=*reservedIndices;
        updateRemovedIndices();
    }
    
    void pcdCloud::setIndices(const pcl::PointIndices indices)
    {
        if(VERBOSE) std::cout << "setting indices...\n";
        *_reservedIndices=indices.indices;
        updateRemovedIndices();
    }

    void pcdCloud::filterValid()
    {
        pcl::PassThrough<RefPointType> pass;
        pass.setInputCloud (_cloud);
        if(!_reservedIndices->empty())
            pass.setIndices(_reservedIndices);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.2, 1.5);
        pass.setKeepOrganized(true);
        pass.filter (*_cloud);
        
        if(_removedIndices->empty())
            _removedIndices->assign(pass.getRemovedIndices()->begin(), pass.getRemovedIndices()->end());
        else
            _removedIndices->insert(_removedIndices->end(), pass.getRemovedIndices()->begin(), pass.getRemovedIndices()->end());
        
        if(VERBOSE)
        {
            if(!_reservedIndices->empty())
            {
                std::cout << "PointCloud before filterValid has: " << _reservedIndices->size() << " data points." << std::endl;
                std::cout<< "filtering out "<< pass.getRemovedIndices()->size() << std::endl;
                std::cout << "PointCloud after filterValid has: " << _reservedIndices->size()-pass.getRemovedIndices()->size() << " valid data points." << std::endl;
            }else
            {
                std::cout << "PointCloud before filterValid has: " << _cloud->points.size() << " data points." << std::endl;
                std::cout<< "filtering out "<< pass.getRemovedIndices()->size() << std::endl;
                std::cout << "PointCloud after filterValid has: " << _cloud->points.size()-pass.getRemovedIndices()->size() << " valid data points." << std::endl;
            }
        }
        
        updateReservedIndices();
    }
    
    void pcdCloud::filterNoise()
    {
        pcl::StatisticalOutlierRemoval<RefPointType> cloud_filter;
        cloud_filter.setInputCloud(_cloud);
        if(!_reservedIndices->empty())
            cloud_filter.setIndices(_reservedIndices);
        cloud_filter.setMeanK(50);
        cloud_filter.setStddevMulThresh(1.0);
        cloud_filter.setKeepOrganized(true);
        cloud_filter.filter(*_cloud);
        
        if(_removedIndices->empty())
            _removedIndices->assign(cloud_filter.getRemovedIndices()->begin(), cloud_filter.getRemovedIndices()->end());
        else
            _removedIndices->insert(_removedIndices->end(), cloud_filter.getRemovedIndices()->begin(), cloud_filter.getRemovedIndices()->end());
        
        
        if(VERBOSE)
        {
            if(!_reservedIndices->empty())
            {
                std::cout << "PointCloud before filterNoise has: " << _reservedIndices->size() << " data points." << std::endl;
                std::cout<< "filtering out "<< cloud_filter.getRemovedIndices()->size() << std::endl;
                std::cout << "PointCloud after filterNoise has: " << _reservedIndices->size()-cloud_filter.getRemovedIndices()->size() << " valid data points." << std::endl;
                
            }else
            {
                std::cout << "PointCloud before filterNoise has: " << _cloud->points.size() << " data points." << std::endl;
                std::cout<< "filtering out "<< cloud_filter.getRemovedIndices()->size() << std::endl;
                std::cout << "PointCloud after filterNoise has: " << _cloud->points.size()-cloud_filter.getRemovedIndices()->size() << " valid data points." << std::endl;
            }
        }
        
        updateReservedIndices();
        
    }
    
    void pcdCloud::extract(CloudPtr output, pcl::IndicesPtr inliers, bool invert=false) 
    {
        _extractor.setInputCloud (_cloud);
        _extractor.setIndices (inliers);
        _extractor.setNegative (invert);
        _extractor.setKeepOrganized(true);
        _extractor.filter (*output);
//         std::cout << "extracting..." << std::endl;
//         std::cout << "received # inliers = " << inliers->size() << std::endl;
//         std::cerr << "extracted point cloud has: " << _reservedIndices->size()-_extractor.getRemovedIndices()->size() << " valid data points." << std::endl;
    }
    
    void pcdCloud::extract(CloudPtr output,  pcl::IndicesPtr inliers, pcl::IndicesPtr outliers, bool invert=false)
    {
        extract(output, inliers, invert);
        outliers->assign(_extractor.getRemovedIndices()->begin(), _extractor.getRemovedIndices()->end());
//         std::cout << "# removedIndices = " << outliers->size() << std::endl;
        // if invert=true, meaning extracting the outliers, then removedIndices which is outliers = inliers 
        if(invert)
        {
            std::cout << "warning: param inliers, outliers are the same\n";
        }
    }
    
    void pcdCloud::findPlane(double threshold)
    {  
        // Create the segmentation object
        pcl::SACSegmentation<RefPointType> seg;
        pcl::PointIndices::Ptr planeInliers(new pcl::PointIndices);
        seg.setOptimizeCoefficients (true); // Optional
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (threshold);
        seg.setInputCloud (_cloud);
        if(!_reservedIndices->empty())
            seg.setIndices(_reservedIndices);
        //std::printf("before seg: _cloud is organized? %d\n", (int)_cloud->isOrganized() ); yes
        seg.segment(*planeInliers, *_coefficients);
        //std::printf("after seg: _cloud is organized? %d\n", (int)_cloud->isOrganized() ); yes
        *_planeInliers = planeInliers->indices;
        
        if (_planeInliers->size() == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            exit(1);
        }     
        
    }
    
    void pcdCloud::extractPlane(CloudPtr inPlaneCloud, CloudPtr outPlaneCloud)
    {
        _extractor.setInputCloud(_cloud);
        _extractor.setIndices(_planeInliers);
        _extractor.setKeepOrganized(true);
        _extractor.setNegative (false);
        _extractor.filter (*inPlaneCloud);
        _extractor.setNegative(true);
        _extractor.filter(*outPlaneCloud);
        
        if(VERBOSE)
        {
            std::cout << "In-plane point cloud has: " << _planeInliers->size() << " valid data points." << std::endl;
            std::cout << "Out-plane point cloud has: " << _reservedIndices->size()-_planeInliers->size() << " valid data points." << std::endl;
        }
    }
    
    void pcdCloud::regionGrow(const bool& colorOn)
    {
        pcl::search::Search<RefPointType>::Ptr tree = boost::shared_ptr<pcl::search::Search<RefPointType> > (new pcl::search::KdTree<RefPointType>);
        pcl::RegionGrowingRGB<RefPointType> reg;
        tree->setInputCloud(_cloud);
        reg.setInputCloud (_cloud);
        if(!_reservedIndices->empty())
            reg.setIndices(_reservedIndices);
        reg.setMinClusterSize (30);
    //     reg.setMaxClusterSize (10000);
        reg.setNumberOfNeighbours (100);
        reg.setSearchMethod (tree);
        
        if(colorOn)
        {
            reg.setDistanceThreshold (0.1);
            reg.setPointColorThreshold (6);
            reg.setRegionColorThreshold (5);
        }else
        {
            reg.setDistanceThreshold (0.05);
        }
        
    //     reg.setNormalTestFlag(true);
    //     reg.setInputNormals (normals);
    //     reg.setSmoothnessThreshold (7.0 / 180.0 * M_PI);
    //     reg.setCurvatureTestFlag(true);
    //     reg.setCurvatureThreshold (1.0);
        
        reg.extract (_clusters);
        _colored_cloud = reg.getColoredCloud ();
    }

    CloudPtr pcdCloud::getCloud() 
    {
        return _cloud;
    }
    
    void pcdCloud::getRemovedIndices(pcl::IndicesPtr removedIndices)
    {
        if(!_removedIndices->empty()){
            *removedIndices = *_removedIndices;
        }else{
            std::cerr << "cannot get _removedIndices since _removedIndices is empty\n";
            exit(1);
        }
            
    }
    
    void pcdCloud::getReservedIndices(pcl::IndicesPtr reservedIndices)
    {
        if(!_reservedIndices->empty()){
            *reservedIndices = *_reservedIndices;
        }else{
            std::cerr << "cannot get _reservedIndices since _reservedIndices is empty\n";
            exit(1);
        }   
    }
    
    void pcdCloud::getPlaneCoefficients(pcl::ModelCoefficients& coefficients)
    {
        if(!_coefficients->values.empty()){
            coefficients = *_coefficients;
            std::cout << "Plane model coefficients: " << _coefficients->values[0] << " " 
                                            << _coefficients->values[1] << " "
                                            << _coefficients->values[2] << " " 
                                            << _coefficients->values[3] << std::endl; 
        }else{
            std::cerr << "cannot get _coefficients since _coefficients is empty\n";
            exit(1);
        }
    }
    
    void pcdCloud::getPlaneInliers(pcl::IndicesPtr planeInliers)
    {
        if(!_planeInliers->empty()){
            *planeInliers = *_planeInliers;
        }else{
            std::cerr << "cannot get _planeInliers since _planeInliers is empty\n";
            exit(1);
        }
    }
    
    void pcdCloud::getClusters(std::vector <pcl::PointIndices>& clusters)
    {
        if(!_clusters.empty()){
            clusters=_clusters;
        }else{
            std::cerr << "cannot get _clusters since _clusters is empty\n";
            exit(1);
        }
    }
    
    void pcdCloud::getThresholdedClusters(std::vector< pcl::PointIndices >& clusters, int threshold)
    {
        for(int i=_clusters.size()-1;i>=0;i--)
        {
            if(_clusters[i].indices.size()<threshold)
                _clusters.erase(_clusters.begin()+i);
        }
        clusters=_clusters;
        
    }

    void pcdCloud::getMaxCluster(pcl::PointIndices::Ptr plane_indices, CloudPtr maxCluster)
    {
        if(!_clusters.empty()){
            int max_size=0;
            int max_index=0;
            for (int i=0; i<_clusters.size(); i++) {
                if (_clusters[i].indices.size()>max_size){
                    max_size = _clusters[i].indices.size();
                    max_index = i;
                }
            }
            
            *plane_indices = _clusters[max_index];
            _extractor.setInputCloud(_cloud);
            _extractor.setIndices(plane_indices);
            _extractor.setKeepOrganized(true);
            _extractor.setNegative (false);
            _extractor.filter (*maxCluster);    
            
            if(VERBOSE) std::cout << "Max point cluster has: " << plane_indices->indices.size() << " valid data points." << std::endl;
        }else{
            std::cerr << "cannot get max cluster since _clusters is empty\n";
            exit(1);
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcdCloud::getColoredCloud()
    {
        return _colored_cloud;
    }

    void pcdCloud::updateRemovedIndices()
    {
        if(!_reservedIndices->empty()){
            CloudPtr temp_cloud(new Cloud);
            extract(temp_cloud, _reservedIndices, _removedIndices);
            
            if(VERBOSE) std::cout << "update: # _removedIndices = " << _removedIndices->size() << std::endl;
        }else{
            std::cerr << "cannot update _removedIndices since _reservedIndices is empty\n";
            exit(1);
        } 
    }
    
    void pcdCloud::updateReservedIndices()
    {
        if(!_removedIndices->empty()){
            CloudPtr temp_cloud(new Cloud);
            extract(temp_cloud, _removedIndices, _reservedIndices);
            
            if(VERBOSE) std::cout << "update: # _reservedIndices = " << _reservedIndices->size() << std::endl;
        }else{
            std::cerr << "cannot update _reservedIndices since _removedIndices is empty\n";
            exit(1);
        } 
    }
    
}
