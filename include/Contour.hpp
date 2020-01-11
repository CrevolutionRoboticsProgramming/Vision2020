#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class Contour
{
public:
    std::vector<cv::Point> pointsVector;
    cv::Rect boundingBox;
    cv::RotatedRect rotatedBoundingBox;
    cv::Point2f rotatedBoundingBoxPoints[4];
    cv::Point2d center;
    double area;
    double angle;

    Contour();
    Contour(std::vector<cv::Point> &pointsVector, int error);
    bool isValid(double minArea, double maxArea, double minContourToBoundingBoxRatio, double maxContourToBoundingBoxRatio);
};
