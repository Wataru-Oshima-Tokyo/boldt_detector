#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include "OpenNI.h"
#include "Detector.h"
#include "Protobuf/Calibration.pb.h"

using namespace cv;

enum DetectorMode {FULL_BIN, EMPTY_BIN};

class DepthDetector : Detector {
public:
    std::vector<cv::Point2i> detect(std::shared_ptr<cv::Mat> img, std::shared_ptr<openni::VideoFrameRef> depthImg);
    std::vector<Point3i> getBestGrabPositions(std::shared_ptr<cv::Mat> img, std::shared_ptr<openni::VideoFrameRef> depthImg);
    DepthDetector(calibration::Calibration calibrationProto);
private:
    std::shared_ptr<cv::Mat> getImageRoiInGreyScale(std::shared_ptr<cv::Mat> img);
    cv::Point2f convertRgbCoordinateToIr(cv::Point2f);
    int getDepthOfIrCoordinate(cv::Point2f, std::shared_ptr<openni::VideoFrameRef> depthImg);
    void updateDetectionMode(std::shared_ptr <openni::VideoFrameRef> depthImg);
    bool doesPointExistInPreviousDetectionBuffer(cv::Point3i);

    int Picking_Range_Row_start;
    int Picking_Range_Row_end;
    int Picking_Range_Column_start;
    int Picking_Range_Column_end;
    int depthThreshold;
    DetectorMode detectorMode = DetectorMode::FULL_BIN;
    double RGB_to_IR_x_a;
    double RGB_to_IR_x_b;
    double RGB_to_IR_x_c;
    double RGB_to_IR_y_a;
    double RGB_to_IR_y_b;
    double RGB_to_IR_y_c;

    std::vector<Point3i> previousDetectionsBuffer;
    int detectionBufferCounter;
    int maxNumberOfGrabPositions;
};
