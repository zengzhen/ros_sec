/**
 * \file        TYPEDEF.h
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       view 3d point cloud in 2d image
 */

#ifndef TYPEDEF
#define TYPEDEF

#include <pcl/point_cloud.h>
#include <pcl/tracking/particle_filter.h>

extern bool VERBOSE;
extern bool DEBUG;
extern bool DEBUG_COLOR;

typedef pcl::PointXYZRGBA RefPointType;
typedef pcl::tracking::ParticleXYZRPY ParticleT;
typedef pcl::tracking::ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
typedef pcl::tracking::ParticleFilterTracker<pcl::PointXYZRGBNormal, ParticleT> ParticleFilterWN;
typedef pcl::PointCloud<RefPointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef const CloudPtr CloudConstPtr;

struct linker{
    std::vector<int> link_index;
    std::vector<float> overlap_ratio;
};
typedef std::vector<linker> linkerList;

struct graph{
    std::vector<int> relational_graph;
    int video_index;
};

struct sec{
    int row;
    std::vector<int> cols;
    std::vector<std::vector<std::string> > event_chain;
    
    void display()
    {
        std::cout << "semantic event chain : " << std::endl;
        std::cout << "row: " << row << std::endl;
        for(int i=0; i<row; i++)
        {
            for(int j=0; j<cols[i]; j++)
            {
                std::string test = event_chain[i][j];
                std::cout << event_chain[i][j].c_str() << " ";
            }
            std::cout << std::endl;
        }
    }
};

#endif  // TYPEDEF