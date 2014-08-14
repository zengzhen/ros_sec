/**
 * \file        track3D.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "track3D.h"

namespace TableObject{
    track3D::track3D(CloudConstPtr target)
    :pcl::tracking::ParticleFilterTracker< RefPointType, ParticleT >()
    {
        _target=target;
        _new_cloud=false;
    }
    
    void track3D::setTarget(CloudConstPtr target)
    {
        _target=target;
    }

    void track3D::initialize()
    {
        iniTrackerParams();
        iniCoherence();
        iniTarget();
    }
    
    void track3D::track(CloudConstPtr inputCloud)
    {
        this->setInputCloud (inputCloud);
        this->compute ();
        std::cout << "change detection " << this->testChangeDetection(inputCloud) << std::endl;
        _new_cloud = true;
    }
    
    void track3D::setNewCloud(bool new_cloud)
    {
        _new_cloud=new_cloud;
    }


    bool track3D::newCloud()
    {
        return _new_cloud;
    }

    void track3D::iniTrackerParams()
    {
        std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
        default_step_covariance[3] *= 40.0;
        default_step_covariance[4] *= 40.0;
        default_step_covariance[5] *= 40.0;

        std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
        std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

        //Set all parameters for  ParticleFilter
        this->setTrans (Eigen::Affine3f::Identity ());
        this->setStepNoiseCovariance (default_step_covariance);
        this->setInitialNoiseCovariance (initial_noise_covariance);
        this->setInitialNoiseMean (default_initial_mean);
        this->setIterationNum (5);
        this->setParticleNum (600);
        this->setResampleLikelihoodThr(0.00);
        this->setUseNormal (true);
    }
    
    void track3D::iniCoherence()
    {
        pcl::tracking::ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence 
        = pcl::tracking::ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
            (new pcl::tracking::ApproxNearestPairPointCloudCoherence<RefPointType> ());
            
        boost::shared_ptr<pcl::tracking::DistanceCoherence<RefPointType> > distance_coherence
            = boost::shared_ptr<pcl::tracking::DistanceCoherence<RefPointType> > 
            (new pcl::tracking::DistanceCoherence<RefPointType> ());
        coherence->addPointCoherence (distance_coherence);

        boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.1));
        coherence->setSearchMethod (search);
        coherence->setMaximumDistance (0.1);

        this->setCloudCoherence (coherence);
    }
    
    void track3D::iniTarget()
    {    
        Eigen::Vector4f c;
        Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
        CloudPtr transed_ref(new Cloud);

        pcl::compute3DCentroid<RefPointType> (*_target, c);
        trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
        pcl::transformPointCloud<RefPointType> (*_target, *transed_ref, trans.inverse());
        
        //set reference model and trans
        this->setReferenceCloud(transed_ref);
        this->setTrans (trans);
    }


}