#pragma once

#include <fstream>
#include <string>
#include <vector>

#include <drake/common/eigen_types.h>
#include <drake/common/text_logging.h>

namespace idto {
namespace optimizer {

using drake::VectorX;
using drake::MatrixX;


template <typename T>
struct LinearizedDynamicsResults {
  std::vector<VectorX<T>> A_lin;
  std::vector<VectorX<T>> B_lin;
};

}  // namespace optimizer
}  // namespace idto

