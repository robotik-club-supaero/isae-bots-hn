#ifndef _POINT_HPP_
#define _POINT_HPP_

#include <functional>
#include <string>

class Point {
   public:
    Point() = default;
    Point(double x, double y);

    bool operator==(Point other) const;

    Point operator+(Point other) const;
    Point operator-(Point other) const;
    Point operator*(double factor) const;
    Point operator/(double factor) const;

    void operator+=(Point other);
    void operator-=(Point other);
    void operator*=(double factor);
    void operator/=(double factor);

    double getX() const;
    double getY() const;

    void setX(double x);
    void setY(double y);

    double getNorm() const;
    Point normalize() const;

    static double dot(Point a, Point b);
    static double det(Point a, Point b);

    operator std::string() const;

   private:
    double m_x;
    double m_y;
};

Point operator*(double factor, Point point);

template <>
struct std::hash<Point> {
    std::size_t operator()(Point point) const {
        std::size_t hash1 = std::hash<double>{}(point.getX());
        std::size_t hash2 = std::hash<double>{}(point.getY());
        return hash1 ^ (hash2 + 0x9e3779b9 + (hash1 << 6) + (hash1 >> 2));
    }
};

#endif