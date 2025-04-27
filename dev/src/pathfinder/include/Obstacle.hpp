#ifndef _OBSTACLE_HPP_
#define _OBSTACLE_HPP_

#include <vector>

#include "Segment.hpp"

class Obstacle {
   public:
    virtual std::vector<Point> getCorners() const = 0;
    virtual bool crosses(Segment segment) const = 0;

    virtual operator std::string() const = 0;
};

class ObstacleCircle : public Obstacle {
   public:
    ObstacleCircle(Point center, double radius);

    std::vector<Point> getCorners() const override;
    bool crosses(Segment point) const override;
    
    Point getCenter() const;
    double getRadius() const;

    void setCenter(Point center);
    void setRadius(double radius);

    operator std::string() const override;

   private:
    Point m_center;
    double m_radius;
};

class ObstacleRect : public Obstacle {
   public:
    ObstacleRect(Point center, double width, double height);
    ObstacleRect(double xMin, double xMax, double yMin, double yMax);

    std::vector<Point> getCorners() const override;
    bool crosses(Segment point) const override;
    bool contains(Point point) const;

    Point getCenter() const;
    double getWidth() const;
    double getHeight() const;

    operator std::string() const override;

   private:
    Point m_topLeft;
    Point m_bottomRight;
};

#endif