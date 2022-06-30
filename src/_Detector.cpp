#include "_Detector.h"
#include <opencv2/dnn.hpp>
#include <iostream>

using namespace cv;
bool roi_captured = false;
Point pt1, pt2;


void mouse_click(int event, int x, int y, int flags, void* param)
{
	switch (event)
	{
	case EVENT_LBUTTONDOWN:
	{
		std::cout << "Mouse Pressed" << std::endl;

		if (!roi_captured)
		{
			pt1.x = x;
			pt1.y = y;
		}
		else
		{
			std::cout << "ROI Already Acquired" << std::endl;
		}
		break;
	}
	case EVENT_LBUTTONUP:
	{
		if (!roi_captured)
		{
			std::cout << "Mouse LBUTTON Released" << std::endl;

			pt2.x = x;
			pt2.y = y;

			roi_captured = true;
		}
		else
		{
			std::cout << "ROI Already Acquired" << std::endl;
		}
		break;
	}

	}
}

_DETECTOR::_DETECTOR()
	: roi(),
	optimalThreshold(0)
    {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

}

_DETECTOR::~_DETECTOR(){
}

std::shared_ptr<Mat> _DETECTOR::getImageRoiInGrayScale(std::shared_ptr<Mat> img) {
	auto temp = *img.get();
	while(!roi_captured)
	{
		namedWindow("roiPrompt", 1);
        setMouseCallback("roiPrompt", mouse_click, 0);
		imshow("roiPrompt", *img.get());
		waitKey(3);
		// if (!)
		// {
		// 	//Wait here till user select the desire ROI
		// 	waitKey(0);
		// }
		//resize(frame, frame, Size(0, 0), 0.7, 0.7); //significantly improves processing time
		
		
	}
    roi = Rect(pt1, pt2);
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

std::vector<Point2i> _DETECTOR::detectBestBolts(std::shared_ptr<Mat> img) {
	std::vector<std::vector<Point> > contours = detectBolts(img, optimalThreshold);
    std::vector<float> nmsWeights;
    std::vector<RotatedRect> rects;
	int i = 0;
	for (auto contour : contours) {
		RotatedRect rect = minAreaRect(contour);
		float height = rect.size.height;
		float width = rect.size.width;
		float aspectRatio = std::min(height / width, width / height);
		int contourArea = cv::contourArea(contour);
        Mat mask(img->size(),CV_8UC1);
        mask = 0;
        cv::drawContours(mask, contours, i, Scalar(255),-1);
        auto mean = cv::mean(*img, mask);
        if (aspectRatio > 0 && contourArea > 0 && mean[0] >= 100) {
            //float weight = aspectRatio + 1000 / (contourArea);
//            float weight = aspectRatio + 255 / mean[0] + 100 / (contourArea);
            float weight = aspectRatio + 255 / mean[0] + 1 / std::max(height, width);
            nmsWeights.push_back(1 / weight);
            rects.push_back(rect);
		}
		i++;
    }

    std::vector<int> idVect;
    cv::dnn::NMSBoxes(rects, nmsWeights, 0.2, 0, idVect);
    std::cout << rects.size() - idVect.size() << std::endl;
    FixedQueue<Point> likeliestGrabPositions(10);
    for (int id : idVect) {
        RotatedRect rect = rects[id];
        float weight = 1 / nmsWeights[id];
        Point center = rect.center;
        center.x += roi.x;
        center.y += roi.y;
        likeliestGrabPositions.push(std::pair<Point, float>(center, weight));
    }
    auto bestPositions = likeliestGrabPositions.getContentVectorAndEmptyQueue();
    return bestPositions;

//	auto bestContours = likeliestContours.getContentVectorAndEmptyQueue();
//	std::vector<Point2f> centroids;
//	for (auto contour : bestContours) {
//		auto mu = moments(contour, false);
//		centroids.push_back(Point2f(mu.m10 / mu.m00 + roi.x, mu.m01 / mu.m00 + roi.y));
//	}
//	destroyAllWindows();
//	return centroids;
}

std::vector<cv::Point2i> _DETECTOR::detect(std::shared_ptr<cv::Mat> img)
{
	auto roiGrey = getImageRoiInGrayScale(img);
	return detectBestBolts(roiGrey);
}

void _DETECTOR::determineOptimalThreshold(std::shared_ptr<Mat> roi)
{
	int optimalNumberOfContours = 50;
	int leastVarianceFromOptimal = 100;
	int threshold = 50;
	for (int i = 0; i < 7; i++) {
		threshold += i * 10;
		auto contours = detectBolts(roi, threshold);
		int numContours = contours.size();
		int varianceFromOptimal = abs(numContours - optimalNumberOfContours);
		if (varianceFromOptimal < leastVarianceFromOptimal) {
			optimalThreshold = threshold;
			leastVarianceFromOptimal = varianceFromOptimal;
		}
	}
	std::cout << "optimal threshold: " << optimalThreshold << std::endl;
}