#include <gflags/gflags.h>
#include "examples/example_base.h"

#include <drake/common/find_resource.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>

namespace idto {
namespace examples {
namespace frictionless_spinner {

DEFINE_string(geometry, "sphere",
              "Geometry of the spinner. Options are 'rectangle', 'capsule', "
              "'square', or 'sphere'.");

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

class FrictionlessSpinnerExample : public TrajOptExample {
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    // N.B. geometry of the spinner is chosen via gflags rather than yaml so
    // that we can use the same yaml format for all of the examples, without
    // cluttering it with spinner-specific options.
    std::string urdf_file;
    if (FLAGS_geometry == "rectangle") {
      urdf_file = idto::FindIDTOResourceOrThrow(
          "examples/models/spinner_rectangle.urdf");
    } else if (FLAGS_geometry == "capsule") {
      urdf_file = idto::FindIDTOResourceOrThrow(
          "examples/models/spinner_capsule.urdf");
    } else if (FLAGS_geometry == "square") {
      urdf_file = idto::FindIDTOResourceOrThrow(
          "examples/models/spinner_square.urdf");
    } else if (FLAGS_geometry == "sphere") {
      urdf_file = idto::FindIDTOResourceOrThrow(
          "examples/models/spinner_sphere.urdf");
    } else {
      throw std::runtime_error(
          fmt::format("Unknown spinner geometry '{}'.", FLAGS_geometry));
    }
    Parser(plant).AddModels(urdf_file);
  }
};

}  // namespace frictionless_spinner
}  // namespace examples
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  idto::examples::frictionless_spinner::FrictionlessSpinnerExample
      example;
  example.RunExample("examples/frictionless_spinner.yaml");
  return 0;
}
