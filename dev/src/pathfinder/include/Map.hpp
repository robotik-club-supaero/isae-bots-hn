#ifndef _MAP_HPP_
#define _MAP_HPP_

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "Obstacle.hpp"

class Map {
   public:
    Map() = default;
    Map(std::unordered_map<std::string, std::shared_ptr<Obstacle>> obstacles);

    std::shared_ptr<Obstacle> getObstacle(const std::string &name) const;
    void setObstacle(const std::string &name, std::shared_ptr<Obstacle> obstacle);
    template <typename T>
    void putObstacle(const std::string &name, T obstacle) {
        setObstacle(name, std::make_shared<T>(std::move(obstacle)));
    }

    std::vector<Point> astarPath(Point from, Point to) const;
    std::unordered_set<Point> getGrid() const;
    bool canGoStraight(Point from, Point to) const;

    void switchToVisibilityGraph();

    template<typename T>
    void switchToRegularGrid(T x_start, T x_end, T y_start, T y_end, T grid_interval);

    operator std::string() const;

   private:
    std::optional<std::unordered_set<Point>> m_grid;
    std::unordered_map<std::string, std::shared_ptr<Obstacle>> m_obstacles;
};

#endif