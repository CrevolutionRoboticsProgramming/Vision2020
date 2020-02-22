#include "Contour.hpp"

Contour::Contour()
{
}

Contour::Contour(std::vector<cv::Point> &pointsVector, int error)
    : pointsVector{pointsVector}
{
    // Approximates a closed polygon with error 3 around the contour and assigns it to newPoly
    cv::Mat newPoly;

    cv::approxPolyDP(cv::Mat(pointsVector), newPoly, error, true);

    // Saves the dimensions of the contour
    area = cv::contourArea(pointsVector);
    rotatedBoundingBox = cv::minAreaRect(newPoly);

    // Draws a rotated rectangle around the contour and assigns its points to points[]
    rotatedBoundingBox.points(rotatedBoundingBoxPoints);

    // Finds the indices of the highest and second-lowest points of the rotated rectangle
    int highestPoint{0}, secondLowestPoint{0};
    for (int i{0}; i < 4; ++i)
    {
        int pointsHigherThan{0}, pointsLowerThan{0};

        for (int o{0}; o < 4; ++o)
        {
            if (rotatedBoundingBoxPoints[i].y < rotatedBoundingBoxPoints[o].y)
                ++pointsHigherThan;
            if (rotatedBoundingBoxPoints[i].y > rotatedBoundingBoxPoints[o].y)
                ++pointsLowerThan;
        }

        if (pointsHigherThan == 3)
            highestPoint = i;
        else if (pointsLowerThan == 2)
            secondLowestPoint = i;
    }

    // Uses the highest and second-lowest points to calculate the rectangle's rotation
    // OpenCV provides a function for this, but it's not consistent with what edge is counted as the top and which indices count for which corners
    angle = std::atan((rotatedBoundingBoxPoints[secondLowestPoint].y - rotatedBoundingBoxPoints[highestPoint].y) / (rotatedBoundingBoxPoints[highestPoint].x - rotatedBoundingBoxPoints[secondLowestPoint].x)) * 180 / 3.1415926;

    // Saves a bounding box for the contour
    boundingBox = cv::boundingRect(newPoly);

    center = cv::Point2d{boundingBox.x + (boundingBox.width / 2.0), boundingBox.y + (boundingBox.height / 2.0)};
}

bool Contour::isValid(double minArea, double maxArea, double minContourToBoundingBoxRatio, double maxContourToBoundingBoxRatio)
{
    // If the area of the contour is not within specification
    if (area < minArea || area > maxArea)
    {
        //std::cout << "Bad area: " << area << '\n';
        return false;
    }

    cv::Point2f points[4];
    rotatedBoundingBox.points(points);

    // Always the smallest
    double height{std::numeric_limits<double>::max()};

    // Always the largest
    double horizontal{std::numeric_limits<double>::min()};

    for (int i{1}; i < 4; ++i)
    {
        double newDistance{std::sqrt(std::pow(points[0].x - points[i].x, 2) + std::pow(points[0].y - points[i].y, 2))};
        if (newDistance < height)
            height = newDistance;
        else if (newDistance > horizontal)
            horizontal = newDistance;
    }

    double width{std::sqrt(std::pow(horizontal, 2) - std::pow(height, 2))};

    double rotatedBoundingBoxArea{width * height};

    // If the ratio of the contour's area to the area of its bounding box is not within specification
    if (area / rotatedBoundingBoxArea < minContourToBoundingBoxRatio || area / rotatedBoundingBoxArea > maxContourToBoundingBoxRatio)
    {
        //std::cout << "Bad ratio " << area / boundingBox.area() << '\n';
        return false;
    }

    return true;
}
