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
		namedWindow("roiPrompt", 1);
		imshow("roiPrompt", *img.get());
		waitKey(3);
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

std::vector<std::vector<Point>> _DETECTOR::detectBolts(std::shared_ptr<Mat> roi, int threshold) {
//	Mat drawing = roi->clone();
    Mat image = *roi.get();
	Mat binary;

    Scalar mean = cv::mean(image);
	float sigma = 0.33;
	int lowerGradThresh = std::max(0, int((1.0 - sigma) * mean[0]));
	int upperGradThresh = std::min(255, int((1.0 + sigma) * mean[0]));

	//cv::threshold(*roi.get(), binary, mean[0] * 1.75, 255, THRESH_BINARY);
	
    Canny(image, binary, lowerGradThresh, upperGradThresh);
//    cv::morphologyEx(binary, binary, MORPH_CLOSE, cv::noArray(), cv::Point(-1, -1), 2);
    cv::dilate(binary, binary, cv::noArray());
    imshow("binary", binary);
    waitKey(1);

	std::vector<std::vector<Point> > contours;
	std::vector<Vec4i> hierarchy;

	/// Detect edges using canny
	/// Find contours
	findContours(binary, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
//	std::cout << contours.size() << std::endl;
//	drawContours(drawing, contours, -1, (100, 255, 255));
//    imshow("contours", drawing);
//    waitKey(0);
	return contours;
}