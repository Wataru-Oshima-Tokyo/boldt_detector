#include "DepthDetector.h"
#include <iostream>
#include "opencv2/highgui.hpp"

using namespace cv;
using namespace std;

std::shared_ptr<cv::Mat> DepthDetector::getImageRoiInGreyScale(std::shared_ptr<cv::Mat> img)
{
    Mat hsv;
    Mat mask;
    cvtColor((*img)(roi), hsv, COLOR_BGR2HSV);
//    inRange(hsv, Scalar(40,0,0), Scalar(80,255,255), mask);
    inRange(hsv, Scalar(86,79,173), Scalar(120,255,255), mask);
    bitwise_not(mask, mask);

    auto m = cv::mean(hsv, mask);
    std::cout << m.val[0] << endl << m.val[1] << endl << m.val[2] << endl;

    auto roiPtr = Detector::getImageRoiInGreyScale(img);
    Mat roiMat = *roiPtr;
    bitwise_and(roiMat, mask, roiMat);

    equalizeHist(roiMat, roiMat);
    GaussianBlur(roiMat, roiMat ,Size(3,3), 0);
    return roiPtr;
}

cv::Point2f DepthDetector::convertRgbCoordinateToIr(cv::Point2f rgbPoint)
{
    int cols = rgbPoint.x;
    int rows = rgbPoint.y;

    float irrows = RGB_to_IR_x_a * rows + RGB_to_IR_x_b * cols + RGB_to_IR_x_c;
    float ircols = RGB_to_IR_y_a * rows + RGB_to_IR_y_b * cols + RGB_to_IR_y_c;

    return cv::Point2f(ircols, irrows);    //  
}

int DepthDetector::getDepthOfIrCoordinate(cv::Point2f irPoint, std::shared_ptr<openni::VideoFrameRef> depthImg)
{
	int irX = irPoint.x;
	int irY = irPoint.y;
    openni::DepthPixel* depthImgRaw = (openni::DepthPixel*)depthImg->getData();  
    int z = depthImgRaw[irX + 640 * irY];
	return z;
}



void DepthDetector::updateDetectionMode(std::shared_ptr<openni::VideoFrameRef> depthImg)
{
    Point2i bottomRowRightRgb(Picking_Range_Column_end, Picking_Range_Row_end);
    Point2i bottomRowLeftRgb(Picking_Range_Column_start, Picking_Range_Row_end);
    Point2i bottomRowRightIr = convertRgbCoordinateToIr(bottomRowRightRgb);
    Point2i bottomRowLeftIr = convertRgbCoordinateToIr(bottomRowLeftRgb);

    int bottomRowNumberIr = bottomRowRightIr.y;
    int bottomRowIrStartX = bottomRowLeftIr.x;
    int bottomRowIrEndX = bottomRowRightIr.x;
    int bottomRowLengthIr = bottomRowIrEndX - bottomRowIrStartX;

    int numPoints = 10;
    int step = bottomRowLengthIr / 10;
    int sumOfDepthDifferences = 0;
    for (int i = 0; i < 10; i++) {
        Point2i currentPoint(bottomRowIrStartX + i * step, bottomRowNumberIr);
        sumOfDepthDifferences += depthThreshold - getDepthOfIrCoordinate(currentPoint, depthImg);
    }

    float averageDepthDifference = sumOfDepthDifferences / 10;
    if (averageDepthDifference < 10) {
        detectorMode = DetectorMode::EMPTY_BIN;
    } else {
        detectorMode = DetectorMode::FULL_BIN;
    }
}



std::vector<cv::Point2i> DepthDetector::detect(std::shared_ptr<cv::Mat> img, std::shared_ptr<openni::VideoFrameRef> depthImg)
{
//    updateDetectionMode(depthImg);
    auto grey = getImageRoiInGreyScale(img);
    return detectBestBolts(grey);
}



static bool depthCompare(Point3f a, Point3f b) {
	return a.z < b.z;
}

std::vector<Point3i> DepthDetector::getBestGrabPositions(std::shared_ptr<cv::Mat> img, std::shared_ptr<openni::VideoFrameRef> depthImg)
{
    FixedQueue <Point3i> highestPoints(maxNumberOfGrabPositions);
    Mat hsv;
    cvtColor(*img, hsv, COLOR_BGR2HSV);
	auto rgbPoints = detect(img, depthImg);
    bool removeLastFramesGrabPositions = rgbPoints.size() > previousDetectionsBuffer.size() + maxNumberOfGrabPositions;
    for (auto rgbPoint : rgbPoints) {
		auto irPoint2D = convertRgbCoordinateToIr(rgbPoint);
        float z = getDepthOfIrCoordinate(irPoint2D, depthImg);
        Point3i currentPoint(rgbPoint.y, rgbPoint.x, z);
        if (removeLastFramesGrabPositions && doesPointExistInPreviousDetectionBuffer(currentPoint)) {
            continue;
        }
        highestPoints.push(std::pair<cv::Point3i, float>(
                               cv::Point3i(rgbPoint.y, rgbPoint.x, z), z)); //point : rgb, rgbY, (ir_column, ir_row, irZ)
    }

    if (detectionBufferCounter++ >= 3) {
        previousDetectionsBuffer.clear();
    }

    if (!highestPoints.empty()) {
        //        return *std::max_element(irPoints3D.begin(), irPoints3D.end(), depthCompare);
        auto points = highestPoints.getContentVectorAndEmptyQueue();
        previousDetectionsBuffer.insert(std::end(previousDetectionsBuffer),
                                        std::begin(points),
                                        std::end(points));
        return points;
    }
    else {
        std::vector<Point3i> emptyVect = {};
        return emptyVect;
    }

}

bool DepthDetector::doesPointExistInPreviousDetectionBuffer(cv::Point3i point){
    int threshold = 10;
    for (Point3i p : previousDetectionsBuffer) {
        if (std::abs(p.x - point.x) < threshold &&
            std::abs(p.y - point.y) < threshold &&
            std::abs(p.z - point.z) < threshold)
        {
            return true;
        }
    }
    return false;
}

DepthDetector::DepthDetector(calibration::Calibration calibrationProto) :
	Detector()
{
	auto const cameraCalibration = calibrationProto.cameracalibration();
    Picking_Range_Row_start = cameraCalibration.picking_range_row_start();
    Picking_Range_Row_end = cameraCalibration.picking_range_row_end();
    Picking_Range_Column_start = cameraCalibration.picking_range_column_start();
    Picking_Range_Column_end = cameraCalibration.picking_range_column_end();
	RGB_to_IR_x_a = cameraCalibration.rgb_to_ir_x_a();
	RGB_to_IR_x_b = cameraCalibration.rgb_to_ir_x_b();
	RGB_to_IR_x_c = cameraCalibration.rgb_to_ir_x_c();
	RGB_to_IR_y_a = cameraCalibration.rgb_to_ir_y_a();
	RGB_to_IR_y_b = cameraCalibration.rgb_to_ir_y_b();
	RGB_to_IR_y_c = cameraCalibration.rgb_to_ir_y_c();
//    depthThreshold = cameraCalibration.depth_threshold();

    roi = cv::Rect(Point(Picking_Range_Column_start, Picking_Range_Row_start),
        Point(Picking_Range_Column_end, Picking_Range_Row_end));
    maxNumberOfGrabPositions = 2;
}
