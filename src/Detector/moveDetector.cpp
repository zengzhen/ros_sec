/**
 * \file        moveDetector.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "ros_sec/Detector/moveDetector.h"
#include "ros_sec/util/util.h"

namespace TableObject{
    moveDetector::moveDetector(float threshold)
    :_cloud(new Cloud)
    {
        _threshold = threshold;
    }

    void moveDetector::setPreCloud(const CloudPtr& cloud)
    {
        *_cloud = *cloud;
    }

        
    bool moveDetector::detect(const CloudPtr& cloud)
    { 
        pcl::PointXYZ pre=TableObject::computeObjCentroid(_cloud);
        pcl::PointXYZ cur=TableObject::computeObjCentroid(cloud);
        float distance = TableObject::distance(pre, cur);
        
        if(VERBOSE)
        {
            std::cout <<"pre: " << pre.x <<" " << pre.y <<" "<<pre.z << std::endl;
            std::cout <<"cur: " << cur.x <<" " << cur.y <<" "<<cur.z << std::endl;
            std::cout <<"distance = " << distance << std::endl;
        }
        
        *_cloud = *cloud;
        if(distance >= _threshold)
        {
            _move = true;
            return true;
        }else{
            _move = false;
            return false;
        }
            
    }

    void moveDetector::showMove(pcl::visualization::PCLVisualizer& viewer, std::string instance, int x)
    {
        std::string instance_move = instance + "_move";
        viewer.removeShape(instance_move);
        std::string move_phrase = instance + ": M";
        std::string nmove_phrase = instance + ": NM";
        if(_move)
        {
            viewer.addText(move_phrase, x, 500, 20, 0,1,0,instance_move);
        }else{
            viewer.addText(nmove_phrase, x, 500,20, 1,0,0,instance_move);
        }
    }


}