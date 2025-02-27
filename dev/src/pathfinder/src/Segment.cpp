#include "Segment.hpp"

Segment::Segment(Point start, Point end) : start(start), end(end) {}

double Segment::getLength() const { return (end - start).getNorm(); }

Point Segment::getDirection() const { return (end - start).normalize(); }