#include <pybind11.h>
#include <stl.h>
#include "BenchmarkAdapter.h"

namespace py=pybind11;

PYBIND11_MODULE(pybench_mr, module) {
    module.doc() = "python binding for bench mr for pytorch motion planner";

    py::class_<BenchmarkAdapter>(module, "BenchmarkAdapterImpl")
            .def(py::init<std::string>())
            .def("collides", &BenchmarkAdapter::collides)
            .def("collides_positions", &BenchmarkAdapter::collides_positions)
            .def("bounds", &BenchmarkAdapter::bounds)
            .def("evaluateAndSaveResult", &BenchmarkAdapter::evaluateAndSaveResult)
            .def("start", &BenchmarkAdapter::start)
            .def("goal", &BenchmarkAdapter::goal)
            .def("evaluatePath", &BenchmarkAdapter::evaluatePath);

    py::class_<Position>(module, "Position")
            .def(py::init<double, double, double>())
            .def_readwrite("x", &Position::x)
            .def_readwrite("y", &Position::y)
            .def_readwrite("angle", &Position::angle);
}

