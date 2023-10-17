#include "examples/example_base.h"
#include <drake/common/find_resource.h>
#include <drake/geometry/proximity_properties.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <gflags/gflags.h>

DEFINE_bool(test, false,
            "whether this example is being run in test mode, where we solve a "
            "simpler problem");

namespace idto {
namespace examples {
namespace allegro_franka {

using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::Vector3d;

/**
 * The Franka arm with an Allegro hand attached manipulates a pen.
 */
class AllegroFrankaExample : public TrajOptExample {
 public:
  AllegroFrankaExample() {
    // Set the camera viewpoint
    const Vector3d camera_pose(1.0, 1.0, 0.3);
    const Vector3d target_pose(0.2, 0.0, 0.2);
    meshcat_->SetCameraPose(camera_pose, target_pose);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const final {
    // Add a model of the hand
    std::string robot_file = idto::FindIdtoResourceOrThrow(
        "idto/examples/models/fr3_algr_minimal_collision.urdf");
    ModelInstanceIndex robot = Parser(plant).AddModels(robot_file)[0];
    plant->set_gravity_enabled(robot, false);

    // Add a model of the pen
    std::string pen_file =
        idto::FindIdtoResourceOrThrow("idto/examples/models/pen.sdf");
    Parser(plant).AddModels(pen_file)[0];
  }
};

}  // namespace allegro_franka
}  // namespace examples
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  idto::examples::allegro_franka::AllegroFrankaExample example;
  example.RunExample("idto/examples/allegro_franka/allegro_franka.yaml",
                     FLAGS_test);

  return 0;
}
