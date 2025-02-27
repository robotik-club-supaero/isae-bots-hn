#include <iostream>

#include "Map.hpp"

int main() {
    Map map;
    map.putObstacle("test", ObstacleCircle(Point(1500, 1500), 223));
    std::cout << map.canGoStraight(Point(999, 501), Point(1500.7, 1799.3)) << std::endl;
    for (auto point : map.astarPath(Point(999, 501), Point(1500.7, 1799.3))) {
        std::cout << std::string(point);
    }
}