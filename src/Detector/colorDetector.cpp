/**
 * \file        colorDetector.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "colorDetector.h"
#include <util/util.h>
#include <iostream>

namespace TableObject{
    
    
    colorDetector::colorDetector(int rl, int rh, int gl, int gh, int bl, int bh)
    :_rl(rl), _rh(rh), _gl(gl), _gh(gh), _bl(bl), _bh(bh), _cloud(new Cloud), _detectedCloud(new Cloud){};
    
    void colorDetector::setInputCloud(const CloudPtr& cloud, const std::vector<pcl::PointIndices>& clusters)
    {
        *_cloud = *cloud;
        _clusters = clusters;
    }

    void colorDetector::filter(pcl::PointIndices& fingertip_ptIdx, CloudPtr& detected_cloud)
    {
        
        //convert 3D color cloud to 2D color image
        cv::Mat image;
        rgbd_vision::convert_rgbd_to_image(_cloud, _clusters, image);
        if(DEBUG_COLOR)
        {
            cv::imshow("frame", image);
            cv::waitKey(-1);
        }
        
        
        // opencv color filtering in 2D images
        cv::Mat threshold(image.rows, image.cols, CV_8UC1);
        cv::inRange(image, cv::Scalar(_bl,_gl,_rl), cv::Scalar(_bh,_gh,_rh),threshold); // scalar(b,g,r)
        if(DEBUG_COLOR)
        {
            cv::imshow("color detection", threshold);
            cv::waitKey(-1);
        }
        
        // find largest connected component with detected color
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(threshold,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
        double max_area=0;
        int fingretip_index;
        for(int i=0; i<contours.size(); i++)
        {
            if(VERBOSE) std::cout << "contour[" << i << "] size = " << cv::contourArea(contours[i]) << std::endl;
            if(cv::contourArea(contours[i]) > max_area)
            {
                max_area = cv::contourArea(contours[i]);
                fingretip_index = i;
            }
        }
        
        // draw detected color object contour
        if(DEBUG_COLOR)
        {
            cv::Mat finger_image(image.rows, image.cols, CV_8UC1, cv::Scalar(0));
            cv::drawContours(finger_image,contours,fingretip_index,cv::Scalar(255),CV_FILLED);
            cv::imshow("fingertip",finger_image);
            cv::waitKey(-1);
        }
        
        // find point indices of detected color object points
        _fingertip_ptIdx.indices.clear();
        for(int r=0; r<threshold.rows; r++)
        {
            for(int c=0;c<threshold.cols; c++)
            {
                cv::Point pt(c,r);
                if(threshold.at<uchar>(r,c)!=0 & cv::pointPolygonTest(contours[fingretip_index],pt,false)>=0)
                {
                    _fingertip_ptIdx.indices.push_back(r*threshold.cols+c);
                }
            }
        }
        std::cout << "fingertip size = " << _fingertip_ptIdx.indices.size() << std::endl;
        
        fingertip_ptIdx = _fingertip_ptIdx;
        
        _detectedCloud.reset(new Cloud);
        TableObject::convertCloud(_cloud, _fingertip_ptIdx, _detectedCloud);
        *detected_cloud = *_detectedCloud;
        
    }

    void colorDetector::showDetectedCloud(pcl::visualization::PCLVisualizer& viewer, std::string cloud_name)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> green(_detectedCloud, 0, 255,0);
        viewer.removePointCloud(cloud_name);
        viewer.addPointCloud<RefPointType>(_detectedCloud, green, cloud_name);
        
    }


}
