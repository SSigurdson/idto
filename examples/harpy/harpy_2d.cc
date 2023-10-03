#include "examples/example_base.h"
#include <drake/common/find_resource.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/prismatic_joint.h>
#include <gflags/gflags.h>

DEFINE_bool(test, false,
            "whether this example is being run in test mode, where we solve a "
            "simpler problem");

namespace idto {
namespace examples {
namespace harpy_2d {

using drake::geometry::Box;
using drake::math::RigidTransformd;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::Vector3d;

/**
 * A simple planar hopper version of the Harpy robot,
 * https://arxiv.org/abs/2004.14337
 */
class Harpy2dExample : public TrajOptExample {
 public:
  Harpy2dExample() {
    meshcat_->Set2dRenderMode();
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    const drake::Vector4<double> green(0.3, 0.6, 0.4, 1.0);

    // Add a the robot
    std::string urdf_file =
        idto::FindIdtoResourceOrThrow("idto/examples/models/harpy_2d.urdf");
    Parser(plant).AddModels(urdf_file);

    // Add collision with the ground
    RigidTransformd X_ground(Vector3d(0.0, 0.0, -5.0));
    plant->RegisterVisualGeometry(plant->world_body(), X_ground,
                                  Box(25, 25, 10), "ground", green);
    plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
                                     Box(25, 25, 10), "ground",
                                     CoulombFriction<double>(0.5, 0.5));
  }

  void CreateCustomMeshcatElements(const TrajOptExampleParams&) const final {
    // Add an arrow showing force on the base
    const drake::geometry::Cylinder cylinder(0.005, 1.0);
    const drake::geometry::Rgba color(1.0, 0.1, 0.1, 1.0);
    meshcat_->SetObject("thruster", cylinder, color);
  }

  void UpdateCustomMeshcatElements(const VectorXd& q, const VectorXd& tau,
                                   const double time) const final {
    // Display thruster forces on th base
    Eigen::Vector3d force(tau[0], 0.0, tau[1]);
    const Eigen::Vector3d origin(q[0], -0.05, q[1]);
    const double height = force.norm();

    RigidTransformd X_WO(origin);
    RigidTransformd X_OC(
        drake::math::RotationMatrixd::MakeFromOneVector(force, 2), 0.5 * force);
    RigidTransformd X_WC = X_WO * X_OC;
    meshcat_->SetProperty("thruster", "scale", {1, 1, height}, time);
    meshcat_->SetTransform("thruster", X_WC, time);
  };
};

}  // namespace harpy_2d
}  // namespace examples
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  idto::examples::harpy_2d::Harpy2dExample example;
  example.RunExample("idto/examples/harpy/harpy_2d.yaml");
  return 0;
}
