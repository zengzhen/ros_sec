/**
 * \file        table_obj_seg.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include "ros_sec/TableObjectSegmentation/table_obj_seg.h"

#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

// #include <pcl/common/centroid.h>
#include "ros_sec/util/util.h"

#include <time.h>

bool TIMING = true;


bool compareClusterSize(const pcl::PointIndices& a,const pcl::PointIndices& b)
{
    return a.indices.size() > b.indices.size();
}

namespace TableObject{
    Segmentation::Segmentation()
    :_sceneCloud(),
    _inPlaneCloud(new Cloud),
    _outPlaneCloud (new Cloud),
    _cloud_hull(new Cloud),
    _tableTopCloud(new Cloud),
    _cloud_objects(new Cloud){};

    Segmentation::Segmentation(const std::string& pcd_file)
    :_sceneCloud(pcd_file),
    _inPlaneCloud(new Cloud),
    _outPlaneCloud (new Cloud),
    _cloud_hull(new Cloud),
    _tableTopCloud(new Cloud),
    _cloud_objects(new Cloud){};
    

    Segmentation::Segmentation(CloudConstPtr cloud)
    :_sceneCloud(cloud),
    _inPlaneCloud(new Cloud),
    _outPlaneCloud (new Cloud),
    _cloud_hull(new Cloud),
    _tableTopCloud(new Cloud),
    _cloud_objects(new Cloud){};
    
    void Segmentation::resetCloud(const std::string& pcd_file, bool downsample)
    {
        _sceneCloud.resetCloud(pcd_file, downsample);
    }

    void Segmentation::resetCloud(CloudConstPtr cloud)
    {
        _sceneCloud.resetCloud(cloud);
    }

    
    void Segmentation::setThreshold(int threshold)
    {
        _threshold=threshold;
    }
    
    void Segmentation::seg_hull(bool view2D)
    {
        if(view2D)
        {
            _viewer2D.viewImage(_sceneCloud.getCloud(), "frame");
            _viewer2D.writeImage("frame.png");
        }
        
        std::clock_t t;
        if(TIMING) t = clock();
        _sceneCloud.filterValid();
//         _sceneCloud.filterNoise();
        _sceneCloud.findPlane(0.01);
        _sceneCloud.getPlaneCoefficients(_coefficients);
        if(TIMING){
            t = clock() - t;
            std::printf("plane segmentation: %f seconds\n", ((float)t)/CLOCKS_PER_SEC);
        }
            
        // extract the inliers of the estimated plane
        _sceneCloud.extractPlane(_inPlaneCloud, _outPlaneCloud);
        if(view2D)
        {
            _viewer2D.viewImage(_inPlaneCloud, "plane");
            _viewer2D.writeImage("plane.png");
        }
        
        // filter small groups of points first before region growing
        pcl::IndicesPtr inliers(new std::vector<int>);
        _sceneCloud.getPlaneInliers(inliers);
        TableObject::pcdCloud inPlanePcdCloud(_inPlaneCloud);
        inPlanePcdCloud.setIndices(inliers);
//         inPlanePcdCloud.filterNoise();
        
        //region grow
        inPlanePcdCloud.regionGrow(false);
        
        //get largest cluster as table top
        pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices);
        _tableTopCloud.reset(new Cloud);
        inPlanePcdCloud.getMaxCluster(plane_indices, _tableTopCloud);
        if(view2D)
        {
            _viewer2D.viewImage(_tableTopCloud, "tabletop");
            _viewer2D.writeImage("tabletop.png");
        }
        
        // Create a Convex Hull representation of the projected inliers
        pcl::ConvexHull<RefPointType> chull;
        chull.setInputCloud (_tableTopCloud);
        chull.setIndices(plane_indices);
        chull.reconstruct (*_cloud_hull);
        
        if(VERBOSE)
        {
            std::cout << "Convex hull has dimemsion of " << chull.getDimension() << std::endl;
            std::cout << "Convex hull has " << _cloud_hull->points.size () << " valid data points." << std::endl;
        }
    }
    
    void Segmentation::seg(bool view2D)
    {
        std::clock_t t;
        if(TIMING) t = clock();
        _sceneCloud.filterValid();
//         _sceneCloud.filterNoise();
        _sceneCloud.findPlane(0.02);
        _sceneCloud.getPlaneCoefficients(_coefficients);
        if(TIMING){
        	t = clock() - t;
        	std::printf("plane segmentation: %f seconds\n", ((float)t)/CLOCKS_PER_SEC);
        }
            
        // extract the inliers of the estimated plane
        _sceneCloud.extractPlane(_inPlaneCloud, _outPlaneCloud);
        if(view2D)
        {
            _viewer2D.viewImage(_sceneCloud.getCloud(), "frame");
            _viewer2D.writeImage("frame.png");
            _viewer2D.viewImage(_inPlaneCloud, "plane");
            _viewer2D.writeImage("plane.png");
            _viewer2D.viewImage(_outPlaneCloud, "out_plane");
            _viewer2D.writeImage("out_plane.png");
        }
        
        // filter small groups of points first before region growing
        pcl::IndicesPtr inliers(new std::vector<int>);
        _sceneCloud.getPlaneInliers(inliers);
        TableObject::pcdCloud inPlanePcdCloud(_inPlaneCloud);
        inPlanePcdCloud.setIndices(inliers);
//         inPlanePcdCloud.filterNoise();
        
        //region grow
        inPlanePcdCloud.regionGrow(false);
        
        //get largest cluster as table top
        pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices);
        _tableTopCloud.reset(new Cloud);
        inPlanePcdCloud.getMaxCluster(plane_indices, _tableTopCloud);
        if(view2D)
        {
            _viewer2D.viewImage(_tableTopCloud, "tabletop");
            _viewer2D.writeImage("tabletop.png");
        }
        
        // Create a Convex Hull representation of the projected inliers
        pcl::ConvexHull<RefPointType> chull;
        chull.setInputCloud (_tableTopCloud);
        chull.setIndices(plane_indices);
        chull.reconstruct (*_cloud_hull);
        
        if(VERBOSE)
        {
            std::cout << "Convex hull has dimemsion of " << chull.getDimension() << std::endl;
            std::cout << "Convex hull has " << _cloud_hull->points.size () << " valid data points." << std::endl;
        }
        
        // get point clouds within planar prism
        pcl::ExtractPolygonalPrismData<RefPointType> prism;
        prism.setInputCloud(_outPlaneCloud);
        prism.setInputPlanarHull(_cloud_hull);
        prism.setHeightLimits(0, 0.5);
        pcl::PointIndices::Ptr output (new pcl::PointIndices);
        prism.segment(*output);
        
        // extract object clouds
        pcl::ExtractIndices<RefPointType> extract;
        extract.setInputCloud (_outPlaneCloud);
        extract.setIndices (output);
        extract.setKeepOrganized(true);
        extract.setNegative (false);
        extract.filter (*_cloud_objects);
        
        if(VERBOSE) std::cout << "Objects point cloud has: " << output->indices.size () << " valid data points." << std::endl;
        
        if(view2D)
        {
            _viewer2D.viewImage(_cloud_objects, "objects");
            _viewer2D.writeImage("objects.png");	
        }
        
        //filter noise in extracted _cloud_objects
        TableObject::pcdCloud objectsPcdCloud(_cloud_objects);
        objectsPcdCloud.setIndices(*output);
//         objectsPcdCloud.filterNoise();
        objectsPcdCloud.regionGrow(false);
        objectsPcdCloud.getThresholdedClusters(_clusters, _threshold);
//         prune();
        
        std::sort(_clusters.begin(), _clusters.end(), compareClusterSize);
        
        if(VERBOSE)
        {
            for(int i=0;i<_clusters.size();i++)
                std::cout << "cluster[" << i << "] has " << _clusters[i].indices.size() << " points\n";
        }
    }
    
    void Segmentation::seg(CloudPtr cloud_hull, bool view2D)
    {
        _cloud_hull=cloud_hull;
        
        std::clock_t t;
        if(TIMING) t=std::clock();
//         _sceneCloud.filterValid();
//         _sceneCloud.filterNoise();
//         _sceneCloud.findPlane(0.005);
        if(TIMING){
        	t = clock() - t;
        	std::printf("plane segmentation: %f seconds\n", ((float)t)/CLOCKS_PER_SEC);
        }
            
        if(TIMING) t = std::clock();
        // extract the inliers of the estimated plane
//         _sceneCloud.extractPlane(_inPlaneCloud, _outPlaneCloud);
        if(view2D)
            _viewer2D.viewImage(_inPlaneCloud, "plane");
        
        // get point clouds within planar prism
        pcl::ExtractPolygonalPrismData<RefPointType> prism;
//         prism.setInputCloud(_outPlaneCloud);
        prism.setInputCloud(_sceneCloud.getCloud());
        prism.setInputPlanarHull(_cloud_hull);
        prism.setHeightLimits(0.005, 0.5);
        pcl::PointIndices::Ptr output (new pcl::PointIndices);
        prism.segment(*output);
        
        // extract object clouds
        pcl::ExtractIndices<RefPointType> extract;
//         extract.setInputCloud (_outPlaneCloud);
        extract.setInputCloud(_sceneCloud.getCloud());
        extract.setIndices (output);
        extract.setKeepOrganized(true);
        extract.setNegative (false);
        extract.filter (*_cloud_objects);
        
        if(TIMING){
        	t = clock() - t;
        	std::printf("extracting object points: %f seconds\n", ((float)t)/CLOCKS_PER_SEC);
        }

        if(VERBOSE) std::cout << "Objects point cloud has: " << output->indices.size () << " valid data points." << std::endl;
        
        if(view2D)
            _viewer2D.viewImage(_cloud_objects, "objects");
        
        if(TIMING) t = std::clock();
        //filter noise in extracted _cloud_objects
        TableObject::pcdCloud objectsPcdCloud(_cloud_objects);
        objectsPcdCloud.setIndices(*output);
        objectsPcdCloud.filterNoise();
        objectsPcdCloud.regionGrow(true);
        objectsPcdCloud.getThresholdedClusters(_clusters, _threshold);
        
        std::sort(_clusters.begin(), _clusters.end(), compareClusterSize);
        
        if(TIMING){
        	t = clock() - t;
        	std::printf("object clustering: %f seconds\n", ((float)t)/CLOCKS_PER_SEC);
        }
        
        if(VERBOSE)
        {
            for(int i=0;i<_clusters.size();i++)
                std::cout << "cluster[" << i << "] has " << _clusters[i].indices.size() << " points\n";
        }
    }
    
    void Segmentation::prune()
    {
        std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > centroids;
        computeObjCentroid(centroids);
        
        for(int i=_clusters.size()-1; i>=0; i--)
        {
            double distance;
            pcl::PointXYZ point;
            Eigen::Vector4f centroid = centroids[i];
            pcl::ModelCoefficients coefficients;
            
            point.x = centroid[1];
            point.y = centroid[2];
            point.z = centroid[3];
            _sceneCloud.getPlaneCoefficients(coefficients);
            distance=pcl::pointToPlaneDistance<pcl::PointXYZ>(point, coefficients.values[0], coefficients.values[1], coefficients.values[2], coefficients.values[3]);
            std::cout << "cluster[" << i << "] has centroid above the tabletop at " << distance << std::endl;
            
            if(distance<0.01)
            {
                std::cout << "cluster[" << i << "] is pruned\n";
                _clusters.erase(_clusters.begin()+i);
            }
        }
    }
    
    void Segmentation::getsceneCloud(pcdCloud& sceneCloud)
    {
        sceneCloud = _sceneCloud;
    }


    void Segmentation::getObjects(CloudPtr cloud_objects, std::vector< pcl::PointIndices >& clusters)
    {
        *cloud_objects=*_cloud_objects;
//         _removedIndices->assign(pass.getRemovedIndices()->begin(), pass.getRemovedIndices()->end());
        clusters.assign(_clusters.begin(), _clusters.end());
    }
    
    void Segmentation::getInPlaneCloud(CloudPtr inPlaneCloud)
    {
        *inPlaneCloud = *_inPlaneCloud;
    }

    void Segmentation::getOutPlaneCloud(CloudPtr outPlaneCloud)
    {
        *outPlaneCloud = *_outPlaneCloud;
    }

    void Segmentation::getCloudHull(CloudPtr cloudHull)
    {
        *cloudHull = *_cloud_hull;
    }

    void Segmentation::getTableTopCloud(CloudPtr tableTopCloud)
    {
        *tableTopCloud = *_tableTopCloud;
    }

	void Segmentation::getPlaneCoefficients(pcl::ModelCoefficients& coefficients)
	{
		coefficients = _coefficients;
	}

    void Segmentation::recordTarget(int cluster_id)
    {
        pcl::PCDWriter writer; 
        CloudPtr target (new Cloud); 
        pcl::copyPointCloud (*_cloud_objects, _clusters[cluster_id], *target); 
        writer.write ("target.pcd", *target, false);
    }
    
    void Segmentation::computeObjCentroid(std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> >& centroid)
    {
        TableObject::computeObjCentroid(_cloud_objects, _clusters, centroid);
    }
    
    
}
