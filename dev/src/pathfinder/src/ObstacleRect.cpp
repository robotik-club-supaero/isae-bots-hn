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

bool line_line_intersect(const Segment &line1, const Segment &line2) {
    Point b = line1.end - line1.start;
    Point d = line2.end - line2.start;
    float bDotDPerp = b.getX() * d.getY() - b.getY() * d.getX();

    // if b dot d == 0, it means the lines are parallel so have infinite intersection points
    if (bDotDPerp == 0) {
        return false;
    }

    Point c = line2.start - line1.start;
    float t = (c.getX() * d.getY() - c.getY() * d.getX()) / bDotDPerp;
    if (t < 0 || t > 1) {
        return false;
    }

    float u = (c.getX() * b.getY() - c.getY() * b.getX()) / bDotDPerp;
    if (u < 0 || u > 1) {
        return false;
    }

    return true;
}

bool ObstacleRect::crosses(Segment segment) const {
    if (contains(segment.start)) {
        if (contains(segment.end)) {
            return true;
        }
        if (Point::dot(segment.end - segment.start, getCenter() - segment.start) < 0) {
            // Enables escape if the robot is already in the obstacle
            return false;
        }
    }

    if (line_line_intersect(segment, Segment(m_topLeft, Point(m_bottomRight.getX(), m_topLeft.getY())))) {
        return true;
    }
    if (line_line_intersect(segment, Segment(m_topLeft, Point(m_topLeft.getX(), m_bottomRight.getY())))) {
        return true;
    }
    if (line_line_intersect(segment, Segment(Point(m_bottomRight.getX(), m_topLeft.getY()), m_bottomRight))) {
        return true;
    }
    if (line_line_intersect(segment, Segment(Point(m_topLeft.getX(), m_bottomRight.getY()), m_bottomRight))) {
        return true;
    }
    return false;
}

bool ObstacleRect::contains(Point point) const {
    return point.getX() > m_topLeft.getX() && point.getX() < m_bottomRight.getX() && point.getY() > m_topLeft.getY() &&
           point.getY() < m_bottomRight.getY();
}
Point ObstacleRect::getCenter() const { return m_topLeft + (m_bottomRight - m_topLeft) / 2; }
double ObstacleRect::getWidth() const { return (m_bottomRight.getX() - m_topLeft.getX()); }
double ObstacleRect::getHeight() const { return (m_bottomRight.getY() - m_topLeft.getY()); }

ObstacleRect::operator std::string() const {
    return "ObstacleRect(center=" + std::string(getCenter()) + ", width=" + std::to_string(getWidth()) + ", height=" + std::to_string(getHeight()) +
           ")";
}