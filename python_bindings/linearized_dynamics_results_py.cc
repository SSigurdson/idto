#include "optimizer/linearized_dynamics_results.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using idto::optimizer::LinearizedDynamicsResults;
using idto::optimizer::ConstraintJacobianResult;

void bind_linearized_dynamics_results(py::module_& m) {
  py::class_<LinearizedDynamicsResults<double>>(m,
                                                  "LinearizedDynamicsResults")
      .def(py::init<>())
      // Traj-opt solution should only be written to by the solver.
      .def_readonly("A_lin", &LinearizedDynamicsResults<double>::A_lin)
      .def_readonly("B_lin", &LinearizedDynamicsResults<double>::B_lin);
}

void bind_constraint_jacobian_result(py::module_& m) {
  py::class_<ConstraintJacobianResult<double>>(m,
                                                  "ConstraintJacobianResult")
      .def(py::init<>())
      .def_readonly("dhdq", &ConstraintJacobianResult<double>::dhdq);
}
