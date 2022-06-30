#include "_Detector.h"
#include <opencv2/dnn.hpp>
#include <iostream>

using namespace cv;
bool roi_captured = false;
Point pt1, pt2;


_DETECTOR::_DETECTOR(){
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

}

_DETECTOR::~_DETECTOR(){
}

std::shared_ptr<Mat> _DETECTOR::getImageRoiInGreyScale(std::shared_ptr<Mat> img) {
	auto temp = *img.get();
	if (roi.size() == Size(0, 0))
	{
		// namedWindow("roiPrompt", 1);
		cv::imshow("roiPrompt", *img.get());
		waitKey(0);
		if (!roi_captured)
		{
			//Wait here till user select the desire ROI
			waitKey(0);
		}
		//resize(frame, frame, Size(0, 0), 0.7, 0.7); //significantly improves processing time
		roi = Rect(pt1, pt2);
		
	}
	Mat gray(roi.size(), CV_8UC3, Scalar(0, 0, 0));
	cvtColor(temp(roi), gray, COLOR_BGR2GRAY);
//    cv::imshow("roi", gray);
//    cv::waitKey(1000);

	return std::make_shared<Mat>(gray);
	//return std::make_shared<Mat>(temp(roi));
}