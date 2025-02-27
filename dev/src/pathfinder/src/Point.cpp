#include "Point.hpp"

#include <cmath>

Point::Point(double x, double y) : m_x(x), m_y(y) {}

bool Point::operator==(Point other) const { return m_x == other.m_x && m_y == other.m_y; }

Point Point::operator+(Point other) const { return Point(m_x + other.m_x, m_y + other.m_y); }
Point Point::operator-(Point other) const { return Point(m_x - other.m_x, m_y - other.m_y); }
Point Point::operator*(double factor) const { return Point(factor * m_x, factor * m_y); }
Point Point::operator/(double factor) const { return Point(m_x / factor, m_y / factor); }

Point operator*(double factor, Point point) { return point * factor; }

void Point::operator+=(Point other) {
    m_x += other.m_x;
    m_y += other.m_y;
}
void Point::operator-=(Point other) {
    m_x -= other.m_x;
    m_y -= other.m_y;
}
void Point::operator*=(double factor) {
    m_x *= factor;
    m_y *= factor;
}
void Point::operator/=(double factor) {
    m_x /= factor;
    m_y /= factor;
}

double Point::getX() const { return m_x; }
double Point::getY() const { return m_y; }

void Point::setX(double x) { m_x = x; }
void Point::setY(double y) { m_y = y; }

double Point::getNorm() const { return std::sqrt(m_x * m_x + m_y * m_y); }

Point Point::normalize() const {
    double norm = getNorm();
    if (norm == 0) {
        return *this;
    }
    return *this / norm;
}

double Point::dot(Point a, Point b) { return a.m_x * b.m_x + a.m_y * b.m_y; }
double Point::det(Point a, Point b) { return a.m_x * b.m_y - a.m_y * b.m_x; }

Point::operator std::string() const { return "Point(" + std::to_string(m_x) + ", " + std::to_string(m_y) + ")"; }