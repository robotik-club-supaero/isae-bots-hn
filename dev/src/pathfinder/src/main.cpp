#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "Map.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pathfinder, m) {
    m.doc() = "Simple A* algorithm using a visibility graph";

    py::class_<Point>(m, "Point")
        .def(py::init<>())
        .def(py::init<double, double>())
        .def("__repr__", &Point::operator std::string)
        .def_property("x", &Point::getX, &Point::setX)
        .def_property("y", &Point::getY, &Point::setY);

    py::class_<Obstacle, std::shared_ptr<Obstacle>>(m, "Obstacle")
        .def("getCorners", &Obstacle::getCorners)
        .def("__repr__", &Obstacle::operator std::string);

    py::class_<ObstacleCircle, Obstacle, std::shared_ptr<ObstacleCircle>>(m, "ObstacleCircle")
        .def(py::init<Point, double>())
        .def_property("center", &ObstacleCircle::getCenter, &ObstacleCircle::setCenter)
        .def_property("radius", &ObstacleCircle::getRadius, &ObstacleCircle::setRadius);

    py::class_<ObstacleRect, Obstacle, std::shared_ptr<ObstacleRect>>(m, "ObstacleRect")
        .def(py::init<Point, double, double>())
        .def(py::init<double, double, double, double>())
        .def_property_readonly("center", &ObstacleRect::getCenter)
        .def_property_readonly("width", &ObstacleRect::getWidth)
        .def_property_readonly("height", &ObstacleRect::getHeight);

    py::class_<Map>(m, "Map")
        .def(py::init<>())
        .def(py::init<std::unordered_map<std::string, std::shared_ptr<Obstacle>>>())
        .def("__repr__", &Map::operator std::string)
        .def("getObstacle", &Map::getObstacle)
        .def("setObstacle", &Map::setObstacle)
        .def("getGrid", &Map::getGrid)
        .def("astarPath", &Map::astarPath);
}