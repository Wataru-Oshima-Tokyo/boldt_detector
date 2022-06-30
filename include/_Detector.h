// #pragma once

#ifndef _DETECTOR_H
#define _DETECTOR_H

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>


 // Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <queue>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>

class _DETECTOR{
    public:
    	std::shared_ptr<cv::Mat> getImageRoiInGreyScale(std::shared_ptr<cv::Mat> img);
    	std::vector<std::vector<cv::Point>> detectBolts(std::shared_ptr<cv::Mat> roi, int threshold);
        std::vector<cv::Point2i> detectBestBolts(std::shared_ptr<cv::Mat> img);
        std::vector<cv::Point2i> detect(std::shared_ptr<cv::Mat> img);
        void determineOptimalThreshold(std::shared_ptr<cv::Mat> roi);
        _DETECTOR();
        ~_DETECTOR();
    protected:
    	cv::Rect roi;
	    int optimalThreshold;
};


#endif //