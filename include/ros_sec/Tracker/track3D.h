/**
 * \file        track3D.h
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       3D tracker given reference cloud
 */

#ifndef TRACK3D_H
#define TRACK3D_H

#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <boost/format.hpp>
#include <Eigen/Geometry>

#include "ros_sec/typeDef.h"

namespace TableObject{
    class track3D: public ParticleFilter {
    public:
        /** \brief empty Constructor
        */
        track3D(){};
        
        /** \brief Constructor
        *  \param[in] target reference cloud for tracking
        */
        track3D(CloudConstPtr target);
        
        /** Set target before initialize
         *  \param[in] target reference cloud for tracking
         */ 
        void setTarget(CloudConstPtr target);
        
        /** Set parameters
         * eg, noise level, particle bins, coherence object for tracking, prepare object for tracking
         */ 
        void initialize();
        
        /** Tracking 
         *  \param[in] inputCloud const pointer to newly arrived point cloud frame
         */
        void track(CloudConstPtr inputCloud);
        
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
        
        
        CloudPtr _target;
        bool _new_cloud;
    };
}

#endif  // TRACK3D_H
