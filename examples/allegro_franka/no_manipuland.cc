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
 * A very simple example involving the Allegro hand attached to the Franka
 * arm, but with no manipuland or interesting contact planning.
 */
class AllegroFrankaNoManipulandExample : public TrajOptExample {
 public:
  AllegroFrankaNoManipulandExample() {
    // Set the camera viewpoint
    const Vector3d camera_pose(3.0, 0.0, 1.0);
    const Vector3d target_pose(0.0, 0.0, 0.5);
    meshcat_->SetCameraPose(camera_pose, target_pose);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const final {
    // Add a model of the hand
    std::string sdf_file = idto::FindIdtoResourceOrThrow(
        "idto/examples/models/fr3_algr_minimal_collision.urdf");
    ModelInstanceIndex robot = Parser(plant).AddModels(sdf_file)[0];
    plant->set_gravity_enabled(robot, false);
  }
};

}  // namespace allegro_franka
}  // namespace examples
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  idto::examples::allegro_franka::AllegroFrankaNoManipulandExample example;
  example.RunExample("idto/examples/allegro_franka/no_manipuland.yaml",
                     FLAGS_test);

  return 0;
}
