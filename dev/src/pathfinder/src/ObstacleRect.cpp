#include <stdexcept>

#include "Obstacle.hpp"

ObstacleRect::ObstacleRect(Point center, double width, double height)
    : m_topLeft(center - Point(width / 2, height / 2)), m_bottomRight(center + Point(width / 2, height / 2)) {
    if (width < 0) {
        throw std::domain_error("width may not be negatve");
    }
    if (height < 0) {
        throw std::domain_error("height may not be negative");
    }
}

ObstacleRect::ObstacleRect(double xMin, double xMax, double yMin, double yMax) : m_topLeft(xMin, yMin), m_bottomRight(xMax, yMax) {}

std::vector<Point> ObstacleRect::getCorners() const {
    return {m_topLeft, m_topLeft + Point(getWidth(), 0), m_bottomRight, m_topLeft + Point(0, getHeight())};
}

bool in_range(double value, double min, double max) { return min <= value && value <= max; }

bool ObstacleRect::crosses(Segment segment) const {
    double xMin = m_topLeft.getX();
    double xMax = m_bottomRight.getX();
    double yMin = m_topLeft.getY();
    double yMax = m_bottomRight.getY();

    double originX = segment.start.getX();
    double originY = segment.start.getY();

    Point lineVect = segment.getDirection();
    if (lineVect.getX() != 0) {
        double tMin = (xMin - originX) / lineVect.getX();
        double tMax = (xMax - originX) / lineVect.getX();

        if ((in_range(originY + tMin * lineVect.getY(), yMin, yMax) && in_range(tMin, 0, 1)) ||
            (in_range(originY + tMax * lineVect.getY(), yMin, yMax) && in_range(tMax, 0, 1))) {
            return true;
        }
    }
    if (lineVect.getY() != 0) {
        double tMin = (yMin - originY) / lineVect.getY();
        double tMax = (yMax - originY) / lineVect.getY();

        if ((in_range(originX + tMin * lineVect.getX(), xMin, xMax) && in_range(tMin, 0, 1)) ||
            (in_range(originX + tMax * lineVect.getX(), xMin, xMax) && in_range(tMax, 0, 1))) {
            return true;
        }
    }
    return false;
}

Point ObstacleRect::getCenter() const { return m_topLeft + (m_bottomRight - m_topLeft) / 2; }
double ObstacleRect::getWidth() const { return (m_bottomRight.getX() - m_topLeft.getX()); }
double ObstacleRect::getHeight() const { return (m_bottomRight.getY() - m_topLeft.getY()); }

ObstacleRect::operator std::string() const {
    return "ObstacleRect(center=" + std::string(getCenter()) + ", width=" + std::to_string(getWidth()) + ", height=" + std::to_string(getHeight()) +
           ")";
}