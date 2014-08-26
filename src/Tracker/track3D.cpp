/**
 * \file        track3D.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "ros_sec/Tracker/track3D.h"

namespace TableObject{
    track3D::track3D(bool use_normal)
    :_tracker_prep(new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8)),
     _tracker_prepWN(new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBNormal, ParticleT> (8)),
     _use_normal(use_normal),
     _target(new Cloud),
     _tracked_cloud(new Cloud),
     _targetWN(new pcl::PointCloud<pcl::PointXYZRGBNormal>)
    {
        _nest.setRadiusSearch (0.01);
    }
    
    void track3D::setTarget(CloudConstPtr target, pcl::PointXYZ center)
    {
        _center=center;
        if(!_use_normal)
            _target=target;
        else{
            // Estimate normals for cloud
            _nest.setInputCloud (target);
            _nest.compute (*_targetWN);
        }
    }

    void track3D::initialize()
    {
        iniTrackerParams();
        iniCoherence();
        iniTarget();
    }
    
    void track3D::track(CloudConstPtr inputCloud, Eigen::Affine3f& transformation)
    {
        if(!_use_normal)
        {
            //Track the object
            _tracker->setInputCloud (inputCloud);        
            _tracker->compute ();
            _new_cloud = true;
            
            pcl::tracking::ParticleXYZRPY result = _tracker->getResult ();
            _transformation = _tracker->toEigenMatrix (result);
            
            pcl::transformPointCloud<RefPointType> (*(_tracker->getReferenceCloud ()), *_tracked_cloud, _transformation);
        }else{
            // Estimate normals for cloud
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloudWN(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            _nest.setInputCloud (inputCloud);
            _nest.compute (*inputCloudWN);
            
            //Track the object
            _trackerWN->setInputCloud (inputCloudWN);        
            _trackerWN->compute ();
            _new_cloud = true;
            
            pcl::tracking::ParticleXYZRPY result = _trackerWN->getResult ();
            _transformation = _trackerWN->toEigenMatrix (result);
        }
        
        transformation = _transformation;
        
        //move center to cylinder center
        transformation.translation () += _offset;
    }
    
    void track3D::getTrackedCloud(CloudPtr tracked_cloud)
    {
        tracked_cloud = _tracked_cloud;
    }
    
    void track3D::viewTrackedCloud (pcl::visualization::PCLVisualizer& viewer)
    {
        if(!_use_normal)
        {
            //Draw blue model reference point cloud
            {
                pcl::visualization::PointCloudColorHandlerCustom<RefPointType> blue_color (_tracked_cloud, 0, 0, 255);

                if (!viewer.updatePointCloud (_tracked_cloud, blue_color, "resultcloud"))
                    viewer.addPointCloud (_tracked_cloud, blue_color, "resultcloud");
            }
        }else{
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::transformPointCloud<pcl::PointXYZRGBNormal> (*(_trackerWN->getReferenceCloud ()), *result_cloud, _transformation);

            //Draw blue model reference point cloud
            {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> blue_color (result_cloud, 0, 0, 255);

                if (!viewer.updatePointCloud (result_cloud, blue_color, "resultcloud"))
                    viewer.addPointCloud (result_cloud, blue_color, "resultcloud");
            }
        }
    }
    
    void track3D::drawParticles (pcl::visualization::PCLVisualizer& viewer)
    {
        ParticleFilter::PointCloudStatePtr particles;
        if(!_use_normal)
            particles = _tracker->getParticles ();
        else
            particles = _trackerWN->getParticles ();
        
        if (_new_cloud)
        {
            std::cout << "Num of particles = " << particles->points.size() << std::endl;
            if (!particles || particles->points.empty ())
                std::cerr << "particles empty\n";
            //Set pointCloud with particle's points
            pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
            for (size_t i = 0; i < particles->points.size (); i++)
            {
                pcl::PointXYZ point;
                    
                point.x = particles->points[i].x;
                point.y = particles->points[i].y;
                point.z = particles->points[i].z;
                particle_cloud->points.push_back (point);
                
                //Debug
//                 std::cout << "particle[" << i << "] = " << point << std::endl;
            }

            //Draw red particles 
            {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color (particle_cloud, 250, 99, 71);

                if (!viewer.updatePointCloud (particle_cloud, red_color, "particle cloud"))
                    viewer.addPointCloud (particle_cloud, red_color, "particle cloud");
                viewer.spinOnce();
            }
        }else{
            std::cerr << "Not ready to draw particles\n";
        }
        _new_cloud = false;
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

        ParticleT bin_size;
        bin_size.x = 0.1f;
        bin_size.y = 0.1f;
        bin_size.z = 0.1f;
        bin_size.roll = 0.1f;
        bin_size.pitch = 0.1f;
        bin_size.yaw = 0.1f;

        if(!_use_normal)
        {
            //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
            _tracker_prep->setMaximumParticleNum (1000);
            _tracker_prep->setDelta (0.99);
            _tracker_prep->setEpsilon (0.2);
            _tracker_prep->setBinSize (bin_size);

            //Set all parameters for  ParticleFilter
            _tracker = _tracker_prep;
            _tracker->setTrans (Eigen::Affine3f::Identity ());
            _tracker->setStepNoiseCovariance (default_step_covariance);
            _tracker->setInitialNoiseCovariance (initial_noise_covariance);
            _tracker->setInitialNoiseMean (default_initial_mean);
            _tracker->setIterationNum (1);
            _tracker->setParticleNum (600);
            _tracker->setResampleLikelihoodThr(0.00);
            _tracker->setUseNormal (_use_normal);
        }else{
            //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
            _tracker_prepWN->setMaximumParticleNum (1000);
            _tracker_prepWN->setDelta (0.99);
            _tracker_prepWN->setEpsilon (0.2);
            _tracker_prepWN->setBinSize (bin_size);

            //Set all parameters for  ParticleFilter
            _trackerWN = _tracker_prepWN;
            _trackerWN->setTrans (Eigen::Affine3f::Identity ());
            _trackerWN->setStepNoiseCovariance (default_step_covariance);
            _trackerWN->setInitialNoiseCovariance (initial_noise_covariance);
            _trackerWN->setInitialNoiseMean (default_initial_mean);
            _trackerWN->setIterationNum (1);
            _trackerWN->setParticleNum (600);
            _trackerWN->setResampleLikelihoodThr(0.00);
            _trackerWN->setUseNormal (_use_normal);
        }
        
        //Debug
//         pcl::tracking::ParticleXYZRPY p;
//         p.zero();
//         p.sample(default_initial_mean, initial_noise_covariance);
//         std::cout << "sampled particle: " << p << std::endl;
//         //prepare the model of tracker's target
//         Eigen::Vector4f c;
//         Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
//         CloudPtr transed_ref(new Cloud);
// 
//         pcl::compute3DCentroid<RefPointType> (*_target, c);
//         trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
//         pcl::tracking::ParticleXYZRPY offset = pcl::tracking::ParticleXYZRPY::toState (trans);
//         std::cout << "offset: " << offset << std::endl;
//         
//         p = p + offset;
//         std::cout << "sampled particle + offset: " << p << std::endl;
    }
    
    void track3D::iniCoherence()
    {
        if(!_use_normal)
        {
            pcl::tracking::ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence 
            = pcl::tracking::ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
                (new pcl::tracking::ApproxNearestPairPointCloudCoherence<RefPointType> ());
            
            // add distance coherence (coherence = 1/ (1+ weight_*diff^2)))
            boost::shared_ptr<pcl::tracking::DistanceCoherence<RefPointType> > distance_coherence
                = boost::shared_ptr<pcl::tracking::DistanceCoherence<RefPointType> > 
                (new pcl::tracking::DistanceCoherence<RefPointType> ());
            distance_coherence->setWeight(2.0);
            coherence->addPointCoherence (distance_coherence);
            
            // add color coherence (coherence = 1/ (1+ weight_*diff^2)))
            boost::shared_ptr<pcl::tracking::HSVColorCoherence<RefPointType> > color_coherence
                = boost::shared_ptr<pcl::tracking::HSVColorCoherence<RefPointType> >
                (new pcl::tracking::HSVColorCoherence<RefPointType> ());
            color_coherence->setWeight(2.0);
            coherence->addPointCoherence (color_coherence);
            
            boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));
            coherence->setSearchMethod (search);
            coherence->setMaximumDistance (0.01);

            _tracker->setCloudCoherence (coherence);
        }else{
            pcl::tracking::ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBNormal>::Ptr coherence 
            = pcl::tracking::ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBNormal>::Ptr
                (new pcl::tracking::ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBNormal> ());
            
            // add distance coherence (coherence = 1/ (1+ weight_*diff^2)))
            boost::shared_ptr<pcl::tracking::DistanceCoherence<pcl::PointXYZRGBNormal> > distance_coherence
                = boost::shared_ptr<pcl::tracking::DistanceCoherence<pcl::PointXYZRGBNormal> > 
                (new pcl::tracking::DistanceCoherence<pcl::PointXYZRGBNormal> ());
            distance_coherence->setWeight(2.0);
            coherence->addPointCoherence (distance_coherence);
            
            // add color coherence (coherence = 1/ (1+ weight_*diff^2)))
            boost::shared_ptr<pcl::tracking::HSVColorCoherence<pcl::PointXYZRGBNormal> > color_coherence
                = boost::shared_ptr<pcl::tracking::HSVColorCoherence<pcl::PointXYZRGBNormal> >
                (new pcl::tracking::HSVColorCoherence<pcl::PointXYZRGBNormal> ());
            color_coherence->setWeight(2.0);
            coherence->addPointCoherence (color_coherence);
            
            boost::shared_ptr<pcl::search::Octree<pcl::PointXYZRGBNormal> > search (new pcl::search::Octree<pcl::PointXYZRGBNormal> (0.01));
            coherence->setSearchMethod (search);
            coherence->setMaximumDistance (0.01);
            
            _trackerWN->setCloudCoherence(coherence);
        }
            
    }
    
    void track3D::iniTarget()
    {            
        //prepare the model of tracker's target
        Eigen::Vector4f c;
        Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
        
        if(!_use_normal)
        {
            CloudPtr transed_ref(new Cloud);
            
            pcl::compute3DCentroid<RefPointType> (*_target, c);
            trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
            pcl::transformPointCloud<RefPointType> (*_target, *transed_ref, trans.inverse());
            
            // Downsample reference cloud
            CloudPtr target_downsampled(new Cloud);
            pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
            float leaf_size=0.005;
            grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
            grid.setInputCloud (transed_ref);
            grid.filter (*target_downsampled);
            
            //set reference model and trans
            _tracker->setReferenceCloud(target_downsampled);
            _tracker->setTrans (trans);
            
            //debug
            if (transed_ref->points.empty ()) 
                    std::cerr << "referenced point cloud empty." << std::endl;
            std::cout << "computed cloud center: " << c << std::endl;
        }else{
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transed_ref(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            
            pcl::compute3DCentroid<pcl::PointXYZRGBNormal> (*_targetWN, c);
            trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
            pcl::transformPointCloud<pcl::PointXYZRGBNormal> (*_targetWN, *transed_ref, trans.inverse());
            
            // Downsample reference cloud
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target_downsampled(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::ApproximateVoxelGrid<pcl::PointXYZRGBNormal> grid;
            float leaf_size=0.005;
            grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
            grid.setInputCloud (transed_ref);
            grid.filter (*target_downsampled);
            
            //set reference model and trans
            _trackerWN->setReferenceCloud(target_downsampled);
            _trackerWN->setTrans (trans);
            
            //debug
            if (transed_ref->points.empty ()) 
                    std::cerr << "referenced point cloud empty." << std::endl;
            std::cout << "computed cloud center: " << c << std::endl;
        }
        
        //calculate offset from cloud center to fitted cylinder center
        // tracker cloud center + _offset = cylinder center
        _offset[0] = _center.x - c[0];
        _offset[1] = _center.y - c[1];
        _offset[2] = _center.z - c[2];
    }


}