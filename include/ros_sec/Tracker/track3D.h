/**
 * \file        track3D.h
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       3D tracker given reference cloud (_use_normal not supported yet in pcl 1.7.2)
 */

#ifndef TRACK3D_H
#define TRACK3D_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/features/normal_3d_omp.h>

#include <boost/format.hpp>
#include <Eigen/Geometry>

#include "ros_sec/typeDef.h"

namespace TableObject{
    class track3D: public ParticleFilter {
    public:
        /** \brief empty Constructor
        */
        track3D(bool use_normal);
        
        /** Set target before initialize
         *  \param[in] target reference cloud for tracking
         *  \param[in] center fitted cylinder center of target cloud
         */ 
        void setTarget(CloudConstPtr target, RefPointType center);
        
        /** Set parameters
         * eg, noise level, particle bins, coherence object for tracking, prepare object for tracking
         */ 
        void initialize();
        
        /** Tracking 
         *  \param[in] inputCloud const pointer to newly arrived point cloud frame
         *  \param[in] transformation transformation.translation = cylinder center, transformation.rotation = relative rotation from initial target cloud to current frame
         */
        void track(CloudConstPtr inputCloud, Eigen::Affine3f& transformation);
        
        /** Get tracked cloud
         *  \param[out] tracked_cloud pointer to tracked cloud
         */
        void getTrackedCloud(CloudPtr& tracked_cloud);
        
        /** Visualize tracked cloud
         *  \param[in] viewer pcl visualizer
         */
        void viewTrackedCloud (pcl::visualization::PCLVisualizer& viewer);
        
        /** Visualize particles
         *  \param[in] viewer pcl visualizer
         */
        void drawParticles (pcl::visualization::PCLVisualizer& viewer);
        
        /** set _new_cloud */
        void setNewCloud(bool new_cloud);
        
        /** Accessor: _new_cloud */
        bool newCloud();
        
    private:
        /** initialize tracker parameters with 
         * default noise level and particle bin size
         */
        void iniTrackerParams();
        
        /** set up coherence object for tracking
         * eg, distance coherence, nearest pair point cloud coherence
         */
        void iniCoherence();
        
        /** set up target cloud for tracking */
        void iniTarget();
        
        boost::shared_ptr<pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > _tracker_prep;
        boost::shared_ptr<ParticleFilter> _tracker;
        CloudPtr _target;
        CloudPtr _tracked_cloud;
        
        boost::shared_ptr<pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBNormal, ParticleT> > _tracker_prepWN;
        boost::shared_ptr<ParticleFilterWN> _trackerWN;
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _targetWN;
        
        bool _new_cloud;
        Eigen::Affine3f _transformation;
        
        RefPointType _center;
        Eigen::Vector3f _offset;
        
        bool _use_normal;
        
        pcl::NormalEstimationOMP<RefPointType,pcl::PointXYZRGBNormal> _nest; 
    };
}

#endif  // TRACK3D_H
