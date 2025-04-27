#include "Map.hpp"

#include <iterator>
#include <queue>
#include <sstream>
#include <stdexcept>

Map::Map(std::unordered_map<std::string, std::shared_ptr<Obstacle>> obstacles) : m_grid(), m_obstacles(std::move(obstacles)) {}

std::shared_ptr<Obstacle> Map::getObstacle(const std::string &name) const {
    auto obs = m_obstacles.find(name);
    if (obs == m_obstacles.end()) {
        return std::shared_ptr<Obstacle>();
    } else {
        return obs->second;
    }
}
void Map::setObstacle(const std::string &name, std::shared_ptr<Obstacle> obstacle) {
    if (!obstacle) {
        m_obstacles.erase(name);
        return;
    }
    m_obstacles[name] = std::move(obstacle);
}

bool Map::canGoStraight(Point from, Point to) const {
    if (from == to) {
        return false;
    }

    Segment segment(from, to);
    for (const auto &obstacle : m_obstacles) {
        if (obstacle.second->crosses(segment)) {
            return false;
        }
    }
    return true;
}

void Map::switchToVisibilityGraph() { m_grid.reset(); }

template <typename T>
void Map::switchToRegularGrid(T x_start, T x_end, T y_start, T y_end, T grid_interval) {
    std::unordered_set<Point> grid;
    for (T x = x_start; x <= x_end; x += grid_interval) {
        for (T y = y_start; y <= y_end; y += grid_interval) {
            grid.insert(Point(x, y));
        }
    }
    m_grid.emplace(std::move(grid));
}
template void Map::switchToRegularGrid<int>(int x_start, int x_end, int y_start, int y_end, int grid_interval);
template void Map::switchToRegularGrid<double>(double x_start, double x_end, double y_start, double y_end, double grid_interval);

std::unordered_set<Point> Map::getGrid() const {
    if (m_grid) {
        return *m_grid;
    } else {
        std::unordered_set<Point> grid;
        for (const auto &obstacle : m_obstacles) {
            std::vector<Point> corners = obstacle.second->getCorners();
            grid.insert(corners.begin(), corners.end());
        }

        return grid;
    }
}

struct State {
    double totalCost;
    Point position;
    std::vector<Point> path;
    double cost;

    bool operator<(const State &other) const { return asTuple() < other.asTuple(); }
    bool operator>(const State &other) const { return other < *this; }

   private:
    std::tuple<double, double, double> asTuple() const { return {totalCost, position.getX(), position.getY()}; }
};

std::vector<Point> Map::astarPath(Point from, Point to) const {
    std::unordered_set<Point> grid = getGrid();
    grid.insert(to);

    std::priority_queue<State, std::vector<State>, std::greater<State>> queue;
    queue.push(State{0, from, {}, 0});

    while (!queue.empty()) {
        State s = queue.top();
        queue.pop();

        grid.erase(s.position);

        if (s.position == to) {
            // Path found
            return s.path;
        }
        for (Point point : grid) {
            if (canGoStraight(s.position, point)) {
                double c = (s.position - point).getNorm() + s.cost;  // Cost of vertex (position -> point)
                double g = c + (point - to).getNorm();               // Total cost: cost + heuristic

                std::vector<Point> path = s.path;
                path.push_back(point);

                queue.push(State{g, point, std::move(path), c});
            }
        }
    }

    return {};
}

Map::operator std::string() const {
    std::ostringstream oss;
    for (const auto &obstacle : m_obstacles) {
        oss << obstacle.first << "=" << std::string(*obstacle.second) << ", ";
    }

    return "Map(obstacles = {" + oss.str() + "})";
}