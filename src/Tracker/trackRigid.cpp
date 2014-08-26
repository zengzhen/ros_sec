/**
 * \file        trackRigid.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "ros_sec/Tracker/trackRigid.h"
#include <pcl/kdtree/kdtree_flann.h>
#include "ros_sec/util/util.h"
#include "ros_sec/Visualizer/view2D.h"
#include <pcl/filters/voxel_grid.h>

#include <pcl/io/pcd_io.h>

namespace TableObject{
    
    trackRigid::trackRigid()
    :_cloud(new Cloud){}
    
    void trackRigid::init(CloudPtr cloud, const std::vector<pcl::PointIndices>& clusters)
    {
        *_cloud = *cloud;
        _clusters = clusters;
    }
    
    void trackRigid::init(CloudPtr cloud, const std::vector< pcl::PointIndices >& clusters, std::vector< RefPointType > clusters_color)
    {
        *_cloud = *cloud;
        _clusters = clusters;
        _clusters_color = clusters_color;
    }
    
    void trackRigid::track_icp(CloudPtr inputCloud, std::vector<pcl::PointIndices>& tracked_clusters)
    {
        // clear _transformed_cloud before tracking, otherwise accumulate
        _transformed_cloud.clear();
        
        // kdtree initialization: Neighbors within radius search
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        
        // ICP for pose estimation and point correspondence
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        icp.setMaximumIterations(2000);
        
        for(int i=0;i<_clusters.size();i++)
        {     
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_targe_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointIndices cluster_i;
            
            // icp_input_cloud: extract each cluster of previous frame cloud to cloud PointXYZRGB
            TableObject::convertCloud(_cloud, _clusters[i], icp_input_cloud, false);
            
            // icp_targe_cloud: convert or extract linked cluster(s) of current frame cloud to cloud PointXYZRGB
            pcl::PointCloud<pcl::PointXYZ>::Ptr kd_tree_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if(_link_to.empty())
            {
                TableObject::convertCloud(inputCloud, icp_targe_cloud, false);
            }else
            {
                // append all linked clusters of _clusteres[i] together into icp_targe_cloud and kd_tree_cloud
                for(int j=0; j<_link_to[i].link_index.size(); j++)
                {
                    int link_index=_link_to[i].link_index[j];
                    TableObject::convertCloud(inputCloud, _link_clusters[link_index], icp_targe_cloud, false);
                    TableObject::convertCloud(inputCloud, _link_clusters[link_index], kd_tree_cloud, true);
                }
            }
           
            //ICP for pose estimation and point correspondence
            icp.setInputSource(icp_input_cloud);
            icp.setInputTarget(icp_targe_cloud);
            icp.align(*icp_transformed_cloud);
            _transformed_cloud.push_back(icp_transformed_cloud);
            _final_transformation = icp.getFinalTransformation ();
            
            std::cout << _final_transformation << std::endl;
            
            if(VERBOSE)
            {
                std::cout << "icp has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
                std::cout << icp.getFinalTransformation() << std::endl;
            }
            
            // kdtree initialization: Neighbors within radius search
            kdtree.setInputCloud(kd_tree_cloud);
            
            // find points close to transformed cloud and color similar to initialized cluster color
            pcl::PointXYZRGB search_point;
            search_point.r = _clusters_color[i].r;
            search_point.g = _clusters_color[i].g;
            search_point.b = _clusters_color[i].b;
            for(int j=0; j<icp_transformed_cloud->points.size(); j++)
            {
                
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;

                float radius = 0.03;
                pcl::PointXYZ ite;
                ite.x = icp_transformed_cloud->points[j].x;
                ite.y = icp_transformed_cloud->points[j].y;
                ite.z = icp_transformed_cloud->points[j].z;
                
                kdtree.radiusSearch(ite, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
                if(!pointIdxRadiusSearch.empty())
                {
                    for(int in_circle_index=0; in_circle_index<pointIdxRadiusSearch.size(); in_circle_index++)
                    {
                        int nearest_index = pointIdxRadiusSearch[in_circle_index];
                        pcl::PointXYZRGBA nearest_point = inputCloud->points[nearest_index];
//                         pcl::PointXYZRGB search_point = icp_transformed_cloud->points[j];
                        
                        if(DEBUG)
                        {
                            std::cout << "search_point: r=" << (int)search_point.r
                                                    <<" g=" << (int)search_point.g
                                                    <<" b=" << (int)search_point.b << std::endl;
                                                    
                            std::cout << "nearest_point: r=" << (int)nearest_point.r
                                                    <<" g=" << (int)nearest_point.g
                                                    <<" b=" << (int)nearest_point.b << std::endl;
                        }
                        
                        float color_distance = TableObject::distance(search_point, nearest_point);
                        if(color_distance<60.0f)
                        {
                            cluster_i.indices.push_back(nearest_index);
                        }
                    }
                }
            }
            
            // remove repeated elemments in cluster_i
            std::set<int> set(cluster_i.indices.begin(), cluster_i.indices.end());
            std::vector<int> cleaner_cluster(set.begin(), set.end());
            cluster_i.indices = cleaner_cluster;
            tracked_clusters.push_back(cluster_i);
            
            std::cout << "tracked_cluster[" << i <<"] size = " << cluster_i.indices.size() << std::endl;
        }
        
        *_cloud = *inputCloud;
        _clusters = tracked_clusters;
    }
    
    void trackRigid::track(CloudPtr inputCloud, std::vector<pcl::PointIndices>& tracked_clusters)
    {
        // clear _transformed_cloud before tracking, otherwise accumulate
        _transformed_cloud.clear();
        
        // kdtree initialization: Neighbors within radius search
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        
        // ICP for pose estimation and point correspondence
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        icp.setMaximumIterations(2000);
        
        for(int i=0;i<_clusters.size();i++)
        {     
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_targe_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointIndices cluster_i;
            
            // icp_input_cloud: extract each cluster of previous frame cloud to cloud PointXYZRGB
            TableObject::convertCloud(_cloud, _clusters[i], icp_input_cloud, false);
            
            // icp_targe_cloud: convert or extract linked cluster(s) of current frame cloud to cloud PointXYZRGB
            pcl::PointCloud<pcl::PointXYZ>::Ptr kd_tree_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if(_link_to.empty())
            {
                TableObject::convertCloud(inputCloud, icp_targe_cloud, false);
            }else
            {
                // append all linked clusters of _clusteres[i] together into icp_targe_cloud and kd_tree_cloud
                for(int j=0; j<_link_to[i].link_index.size(); j++)
                {
                    int link_index=_link_to[i].link_index[j];
                    TableObject::convertCloud(inputCloud, _link_clusters[link_index], icp_targe_cloud, false);
                    TableObject::convertCloud(inputCloud, _link_clusters[link_index], kd_tree_cloud, true);
                }
            }
            
            // Downsample
            pcl::console::print_highlight ("Downsampling...\n");
            pcl::VoxelGrid<pcl::PointXYZRGB> grid;
            const float leaf = 0.005f;
            grid.setLeafSize (leaf, leaf, leaf);
            grid.setInputCloud (icp_input_cloud);
            grid.filter (*icp_input_cloud);
            grid.setInputCloud (icp_targe_cloud);
            grid.filter (*icp_targe_cloud);
            
            // point normal clouds
            PointCloudT::Ptr object (new PointCloudT);
            PointCloudT::Ptr scene (new PointCloudT);
            FeatureCloudT::Ptr object_features (new FeatureCloudT);
            FeatureCloudT::Ptr scene_features (new FeatureCloudT);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_aligned (new pcl::PointCloud<pcl::PointXYZRGB>);
            
            // Estimate normals for scene
            pcl::console::print_highlight ("Estimating target cloud normals...\n");
            pcl::NormalEstimationOMP<pcl::PointXYZRGB,PointNT> nest;
            nest.setRadiusSearch (0.01);
            nest.setInputCloud (icp_targe_cloud);
            nest.compute (*scene);
            nest.setInputCloud (icp_input_cloud);
            nest.compute (*object);

            // Estimate features
            pcl::console::print_highlight ("Estimating features...\n");
            FeatureEstimationT fest;
            fest.setRadiusSearch (0.025);
            fest.setInputCloud (icp_input_cloud);
            fest.setInputNormals (object);
            fest.compute (*object_features);
            fest.setInputCloud (icp_targe_cloud);
            fest.setInputNormals (scene);
            fest.compute (*scene_features);

            // Perform alignment
            pcl::console::print_highlight ("Starting alignment...\n");
            pcl::SampleConsensusPrerejective<pcl::PointXYZRGB,pcl::PointXYZRGB,FeatureT> align;
            align.setInputSource (icp_input_cloud);
            align.setSourceFeatures (object_features);
            align.setInputTarget (icp_targe_cloud);
            align.setTargetFeatures (scene_features);
            align.setMaximumIterations (10000); // Number of RANSAC iterations
            align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
            align.setCorrespondenceRandomness (2); // Number of nearest features to use
            align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
            align.setMaxCorrespondenceDistance (1.5f*leaf); // Inlier threshold
            align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
            {
                pcl::ScopeTime t("Alignment");
                align.align (*object_aligned);
            }
            if (align.hasConverged ())
            {
                // Print results
                _final_transformation = align.getFinalTransformation ();
                std::cout << _final_transformation << std::endl;
                std::cout << "transformed_cloud is organized: " << object_aligned->isOrganized() << std::endl;
            }else
            {
                pcl::console::print_error ("Alignment failed!\n");
            }
            
            // kdtree initialization: Neighbors within radius search
            kdtree.setInputCloud(kd_tree_cloud);
            
            // find points close to transformed cloud and color similar to initialized cluster color
            pcl::PointXYZRGB search_point;
            search_point.r = _clusters_color[i].r;
            search_point.g = _clusters_color[i].g;
            search_point.b = _clusters_color[i].b;
            for(int j=0; j<object_aligned->points.size(); j++)
            {
                
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;

                float radius = 0.03;
                pcl::PointXYZ ite;
                ite.x = object_aligned->points[j].x;
                ite.y = object_aligned->points[j].y;
                ite.z = object_aligned->points[j].z;
                
                kdtree.radiusSearch(ite, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
                if(!pointIdxRadiusSearch.empty())
                {
                    for(int in_circle_index=0; in_circle_index<pointIdxRadiusSearch.size(); in_circle_index++)
                    {
                        int nearest_index = pointIdxRadiusSearch[in_circle_index];
                        pcl::PointXYZRGBA nearest_point = inputCloud->points[nearest_index];
//                         pcl::PointXYZRGB search_point = icp_transformed_cloud->points[j];
                        
                        if(DEBUG)
                        {
                            std::cout << "search_point: r=" << (int)search_point.r
                                                    <<" g=" << (int)search_point.g
                                                    <<" b=" << (int)search_point.b << std::endl;
                                                    
                            std::cout << "nearest_point: r=" << (int)nearest_point.r
                                                    <<" g=" << (int)nearest_point.g
                                                    <<" b=" << (int)nearest_point.b << std::endl;
                        }
                        
                        float color_distance = TableObject::distance(search_point, nearest_point);
                        if(color_distance<60.0f)
                        {
                            cluster_i.indices.push_back(nearest_index);
                        }
                    }
                }
            }
            
            // remove repeated elemments in cluster_i
            std::set<int> set(cluster_i.indices.begin(), cluster_i.indices.end());
            std::vector<int> cleaner_cluster(set.begin(), set.end());
            cluster_i.indices = cleaner_cluster;
            tracked_clusters.push_back(cluster_i);
            
            std::cout << "tracked_cluster[" << i <<"] size = " << cluster_i.indices.size() << std::endl;
        }
        
        *_cloud = *inputCloud;
        _clusters = tracked_clusters;
    }
    
    void trackRigid::track_debug(CloudPtr inputCloud, std::vector<pcl::PointIndices>& tracked_clusters, bool save)
    {
        // clear _transformed_cloud before tracking, otherwise accumulate
        _transformed_cloud.clear();
        
        // kdtree initialization: Neighbors within radius search
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        
        // ICP for pose estimation and point correspondence
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        icp.setMaximumIterations(2000);
        
        for(int i=0;i<_clusters.size();i++)
        {     
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_targe_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointIndices cluster_i;
            
            // icp_input_cloud: extract each cluster of previous frame cloud to cloud PointXYZRGB
            TableObject::convertCloud(_cloud, _clusters[i], icp_input_cloud, false);
            
            // icp_targe_cloud: convert or extract linked cluster(s) of current frame cloud to cloud PointXYZRGB
            pcl::PointCloud<pcl::PointXYZ>::Ptr kd_tree_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if(_link_to.empty())
            {
                TableObject::convertCloud(inputCloud, icp_targe_cloud, false);
            }else
            {
                // append all linked clusters of _clusteres[i] together into icp_targe_cloud and kd_tree_cloud
                for(int j=0; j<_link_to[i].link_index.size(); j++)
                {
                    int link_index=_link_to[i].link_index[j];
                    TableObject::convertCloud(inputCloud, _link_clusters[link_index], icp_targe_cloud, false);
                    TableObject::convertCloud(inputCloud, _link_clusters[link_index], kd_tree_cloud, true);
                }
            }
            
            // save pcd for debug
            if(save)
            {
                pcl::io::savePCDFileASCII("bottle_input.pcd", *icp_input_cloud);
                pcl::io::savePCDFileASCII("bottle_target.pcd", *icp_targe_cloud);
            }
            
            // Downsample
            pcl::console::print_highlight ("Downsampling...\n");
            pcl::VoxelGrid<pcl::PointXYZRGB> grid;
            const float leaf = 0.005f;
            grid.setLeafSize (leaf, leaf, leaf);
            grid.setInputCloud (icp_input_cloud);
            grid.filter (*icp_input_cloud);
            grid.setInputCloud (icp_targe_cloud);
            grid.filter (*icp_targe_cloud);
            
            // point normal clouds
            PointCloudT::Ptr object (new PointCloudT);
            PointCloudT::Ptr scene (new PointCloudT);
            FeatureCloudT::Ptr object_features (new FeatureCloudT);
            FeatureCloudT::Ptr scene_features (new FeatureCloudT);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_aligned (new pcl::PointCloud<pcl::PointXYZRGB>);
            
            // Estimate normals for scene
            pcl::console::print_highlight ("Estimating target cloud normals...\n");
            pcl::NormalEstimationOMP<pcl::PointXYZRGB,PointNT> nest;
            nest.setRadiusSearch (0.01);
            nest.setInputCloud (icp_targe_cloud);
            nest.compute (*scene);
            nest.setInputCloud (icp_input_cloud);
            nest.compute (*object);

            // Estimate features
            pcl::console::print_highlight ("Estimating features...\n");
            FeatureEstimationT fest;
            fest.setRadiusSearch (0.025);
            fest.setInputCloud (icp_input_cloud);
            fest.setInputNormals (object);
            fest.compute (*object_features);
            fest.setInputCloud (icp_targe_cloud);
            fest.setInputNormals (scene);
            fest.compute (*scene_features);

            // Perform alignment
            pcl::console::print_highlight ("Starting alignment...\n");
            pcl::SampleConsensusPrerejective<pcl::PointXYZRGB,pcl::PointXYZRGB,FeatureT> align;
            align.setInputSource (icp_input_cloud);
            align.setSourceFeatures (object_features);
            align.setInputTarget (icp_targe_cloud);
            align.setTargetFeatures (scene_features);
            align.setMaximumIterations (15000); // Number of RANSAC iterations
            align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
            align.setCorrespondenceRandomness (2); // Number of nearest features to use
            align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
            align.setMaxCorrespondenceDistance (1.5f*leaf); // Inlier threshold
            align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
            {
                pcl::ScopeTime t("Alignment");
                align.align (*object_aligned);
            }
            if (align.hasConverged ())
            {
                // Print results
                _final_transformation = align.getFinalTransformation ();
                std::cout << _final_transformation << std::endl;
                std::cout << "transformed_cloud is organized: " << object_aligned->isOrganized() << std::endl;
            }else
            {
                pcl::console::print_error ("Alignment failed!\n");
            }
            
            // kdtree initialization: Neighbors within radius search
            kdtree.setInputCloud(kd_tree_cloud);
            
            // find points close to transformed cloud and color similar to initialized cluster color
            pcl::PointXYZRGB search_point;
            search_point.r = _clusters_color[i].r;
            search_point.g = _clusters_color[i].g;
            search_point.b = _clusters_color[i].b;
            for(int j=0; j<object_aligned->points.size(); j++)
            {
                
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;

                float radius = 0.03;
                pcl::PointXYZ ite;
                ite.x = object_aligned->points[j].x;
                ite.y = object_aligned->points[j].y;
                ite.z = object_aligned->points[j].z;
                
                kdtree.radiusSearch(ite, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
                if(!pointIdxRadiusSearch.empty())
                {
                    for(int in_circle_index=0; in_circle_index<pointIdxRadiusSearch.size(); in_circle_index++)
                    {
                        int nearest_index = pointIdxRadiusSearch[in_circle_index];
                        pcl::PointXYZRGBA nearest_point = inputCloud->points[nearest_index];
//                         pcl::PointXYZRGB search_point = icp_transformed_cloud->points[j];
                        
                        if(DEBUG)
                        {
                            std::cout << "search_point: r=" << (int)search_point.r
                                                    <<" g=" << (int)search_point.g
                                                    <<" b=" << (int)search_point.b << std::endl;
                                                    
                            std::cout << "nearest_point: r=" << (int)nearest_point.r
                                                    <<" g=" << (int)nearest_point.g
                                                    <<" b=" << (int)nearest_point.b << std::endl;
                        }
                        
                        float color_distance = TableObject::distance(search_point, nearest_point);
                        if(color_distance<60.0f)
                        {
                            cluster_i.indices.push_back(nearest_index);
                        }
                    }
                }
            }
            
            // remove repeated elemments in cluster_i
            std::set<int> set(cluster_i.indices.begin(), cluster_i.indices.end());
            std::vector<int> cleaner_cluster(set.begin(), set.end());
            cluster_i.indices = cleaner_cluster;
            tracked_clusters.push_back(cluster_i);
            
            std::cout << "tracked_cluster[" << i <<"] size = " << cluster_i.indices.size() << std::endl;
        }
        
        *_cloud = *inputCloud;
        _clusters = tracked_clusters;
    }
    
    void trackRigid::linking(CloudPtr inputCloud, const std::vector< pcl::PointIndices >& inputClusters)
    {
        
        _link_to = TableObject::linking(_cloud, _clusters, inputCloud, inputClusters);
        _link_clusters = inputClusters;
    }

    
    void trackRigid::setLinkTo(const linkerList& link_to, const std::vector<pcl::PointIndices>& link_clusters)
    {
        _link_to = link_to;
        _link_clusters = link_clusters;
    }
    
    void trackRigid::getTransformedCloud(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& transformed_cloud)
    {
        transformed_cloud = _transformed_cloud;
    }
    
    void trackRigid::getFinalTransformation(Eigen::Matrix4f & transformation)
    {
        transformation = _final_transformation;
    }
    
    void trackRigid::getTestClusters(std::vector<pcl::PointIndices>& clusters)
    {
        clusters = _clusters;
    }
    
    void trackRigid::viewTranformedCloud(pcl::visualization::PCLVisualizer& viewer, int cluster_id)
    {
        std::stringstream tc ("transformed"); 
        tc <<cluster_id; 
        
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green (_transformed_cloud[cluster_id], 0,255,0);
        if(!viewer.updatePointCloud<pcl::PointXYZRGB>(_transformed_cloud[cluster_id], green, tc.str())) viewer.addPointCloud<pcl::PointXYZRGB>(_transformed_cloud[cluster_id], green, tc.str());
    }

    void trackRigid::viewTrackedCloud(pcl::visualization::PCLVisualizer& viewer, int cluster_id, int r, int g, int b)
    {
        std::stringstream tc ("cluster"); 
        tc <<cluster_id;
        
        CloudPtr tracked_cloud(new Cloud);
        pcl::copyPointCloud(*_cloud, _clusters[cluster_id], *tracked_cloud);
        pcl::visualization::PointCloudColorHandlerCustom<RefPointType> color (tracked_cloud, r,g,b); 
        if(!viewer.updatePointCloud<RefPointType>(tracked_cloud, color, tc.str()))
        {
            viewer.addPointCloud<RefPointType>(tracked_cloud, color, tc.str());
        }
    }

}
