#pragma once
#include <opencv2/core/core.hpp>
#include <memory>
#include <queue>
#include <cmath>
#include <algorithm>

template <class V> struct LessThanByAspectRatio
{
    bool operator()(const std::pair<V, float>& lhs, const std::pair<V, float>& rhs) const
    {
        return lhs.second < rhs.second;
    }
};

template <class T> class FixedQueue {
private:
    std::priority_queue<std::pair<T, float>,
        std::vector<std::pair<T, float>>,
        LessThanByAspectRatio<T>> queue;
    int maxSize;
public:
    FixedQueue(int size)
        : queue()
    {
        maxSize = size;
    }
    std::pair<T, float> pop() {
        auto top = queue.top();
        queue.pop();
        return top;
    }
    void push(std::pair<T, float> obj) {
        queue.emplace(obj);
        if (queue.size() > maxSize) {
            queue.pop();
        }
    }
    std::vector<T> getContentVectorAndEmptyQueue() {
        std::vector<T> vect;
        int numElements = queue.size();
        for (int i = 0; i < numElements; i++) {
            vect.push_back(pop().first);
        }
        return vect;
    }
    bool empty() {
        return queue.empty();
    }
};

class Detector
{
public:
	std::shared_ptr<cv::Mat> getImageRoiInGreyScale(std::shared_ptr<cv::Mat> img);
	std::vector<std::vector<cv::Point>> detectBolts(std::shared_ptr<cv::Mat> roi, int threshold);
    std::vector<cv::Point2i> detectBestBolts(std::shared_ptr<cv::Mat> img);
    std::vector<cv::Point2i> detect(std::shared_ptr<cv::Mat> img);
	void determineOptimalThreshold(std::shared_ptr<cv::Mat> roi);
	Detector();
    ~Detector();
protected:
	cv::Rect roi;
	int optimalThreshold;
};

