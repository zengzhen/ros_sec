/**
 * \file        moveDetector.h
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       detect whether an object moves
 */

#ifndef MOVEDETECTOR_H
#define MOVEDETECTOR_H

#include "typeDef.h"
#include <pcl/visualization/pcl_visualizer.h>

namespace TableObject{
    
    class moveDetector {
        
    public:
        moveDetector(float threshold);
        
        void setPreCloud(const CloudPtr& cloud);
        
        bool detect(const CloudPtr& cloud);
        
        void showMove(pcl::visualization::PCLVisualizer& viewer, std::string instance, int x);
        
        
        
    private:
        CloudPtr _cloud;
        float _threshold;
        bool _move;
    };
}

#endif //MOVEDETECTOR_H