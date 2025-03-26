#include <stdexcept>
#include <cmath>

#include "Obstacle.hpp"

inline void checkRadius(double radius) {
    if (radius < 0) {
        throw std::domain_error("radius must not be negative");
    }
}

ObstacleCircle::ObstacleCircle(Point center, double radius) : m_center(center), m_radius(radius) { checkRadius(radius); }

std::vector<Point> ObstacleCircle::getCorners() const {
    Point radX(0.9 * m_radius, 0);
    Point radY(0, 0.9 * m_radius);

    return {m_center + radX + radY, m_center + radX - radY, m_center - radX + radY, m_center - radX - radY};
}

bool ObstacleCircle::crosses(Segment segment) const {
    Point OA = segment.start - m_center;
    Point OB = segment.end - m_center;
    if (Point::dot(OA, segment.start - segment.end) > 0 && Point::dot(OB, segment.end - segment.start) > 0) {
        return std::abs(Point::det(OA, OB)) / segment.getLength() < m_radius;
    } else {
        return std::min(OA.getNorm(), OB.getNorm()) < m_radius;
    }
}

Point ObstacleCircle::getCenter() const { return m_center; }
double ObstacleCircle::getRadius() const { return m_radius; }

void ObstacleCircle::setCenter(Point center) { m_center = center; }
void ObstacleCircle::setRadius(double radius) { m_radius = radius; }

ObstacleCircle::operator std::string() const {
    return "ObstacleCircle(center=" + std::string(m_center) + ", radius=" + std::to_string(m_radius) + ")";
}