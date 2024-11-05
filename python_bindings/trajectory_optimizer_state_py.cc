#include <iostream>

#include <vector>

#include "optimizer/trajectory_optimizer_state.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

using drake::VectorX;
using idto::optimizer::TrajectoryOptimizerState;


void bind_trajectory_optimizer_state(py::module_& m) {
  py::class_<TrajectoryOptimizerState<double>>(m, "TrajectoryOptimizerState")
      .def("q", &TrajectoryOptimizerState<double>::q)
      .def("set_q",
           [](TrajectoryOptimizerState<double>& state,
              const std::vector<VectorX<double>>& q) {
             state.set_q(q);
           });
}
