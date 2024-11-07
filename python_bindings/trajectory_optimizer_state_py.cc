#include <iostream>

#include <vector>

#include "optimizer/trajectory_optimizer_state.h"
#include "optimizer/penta_diagonal_matrix.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#define PYBIND11_DETAILED_ERROR_MESSAGES

namespace py = pybind11;

using drake::VectorX;
using drake::MatrixX;
using idto::optimizer::TrajectoryOptimizerState;
using idto::optimizer::PentaDiagonalMatrix;


void bind_trajectory_optimizer_state(py::module_& m) {
  py::class_<TrajectoryOptimizerState<double>>(m, "TrajectoryOptimizerState")
      .def("q", &TrajectoryOptimizerState<double>::q)
      .def("set_q",
           [](TrajectoryOptimizerState<double>& state,
              const std::vector<VectorX<double>>& q) {
             state.set_q(q);
           });
}

void bind_penta_diagonal_matrix(py::module_& m) {
  py::class_<PentaDiagonalMatrix<double>>(m, "PentaDiagonalMatrix")
      .def("MakeDense", &PentaDiagonalMatrix<double>::MakeDense, py::return_value_policy::reference);
}
