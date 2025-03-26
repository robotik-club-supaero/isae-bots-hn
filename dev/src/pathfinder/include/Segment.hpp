#ifndef _SEGMENT_HPP_
#define _SEGMENT_HPP_

#include "Point.hpp"

class Segment {
   public:
    Segment(Point start, Point end);

    double getLength() const;
    Point getDirection() const;

    Point start;
    Point end;
};

#endif