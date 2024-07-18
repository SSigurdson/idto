#!/usr/bin/env python

##
#
# A simple sanity check with Adrian's Achilles humanoid.
#
##

from pydrake.all import (
    StartMeshcat,
    DiagramBuilder,
    AddMultibodyPlantSceneGraph,
    AddDefaultVisualization,
    Parser,
    Box,
    RigidTransform,
    CoulombFriction,
)
import time
import numpy as np

from pyidto import (
    TrajectoryOptimizer,
    TrajectoryOptimizerSolution,
    TrajectoryOptimizerStats,
    SolverParameters,
    ProblemDefinition,
    FindIdtoResource,
)

def visualize_trajectory(q, time_step, model_file, meshcat=None):
    """
    Display the given trajectory (list of configurations) on meshcat
    """
    # Create a simple Drake diagram with a plant model
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step)
    Parser(plant).AddModels(model_file)
    plant.Finalize()

    # Connect to the meshcat visualizer
    AddDefaultVisualization(builder, meshcat)

    # Build the system diagram
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
    plant.get_actuation_input_port().FixValue(plant_context,
                                              np.zeros(plant.num_actuators()))

    # Step through q, setting the plant positions at each step
    meshcat.StartRecording()
    for k in range(len(q)):
        diagram_context.SetTime(k * time_step)
        plant.SetPositions(plant_context, q[k])
        diagram.ForcedPublish(diagram_context)
        time.sleep(time_step)
    meshcat.StopRecording()
    meshcat.PublishRecording()

if __name__=="__main__":
    model_file = FindIdtoResource("idto/models/achilles/achilles.urdf")

    # Create the system diagram that the optimizer uses
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.05)
    Parser(plant).AddModels(model_file)
    plant.RegisterCollisionGeometry(
        plant.world_body(), 
        RigidTransform(p=[0, 0, -25]), 
        Box(50, 50, 50), "ground", 
        CoulombFriction(0.5, 0.5))
    plant.Finalize()  # TODO: add a ground
    diagram = builder.Build()

    nq = plant.num_positions()
    nv = plant.num_velocities()
    q_nom = np.array([
        1.0, 0.0, 0.08, 0.0,         # base orientation
        0.0, 0.0, 0.93,              # base position
        0.0, 0.0, -0.7, 1.0, -0.45,  # left leg
        0.0, 0.0, 0.0, -0.9,         # left arm
        0.0, 0.0, -0.7, 1.0, -0.45,  # left leg
        0.0, 0.0, 0.0, -0.9,         # right arm
    ])

    # Specify a cost function and target trajectory
    problem = ProblemDefinition()
    problem.num_steps = 20
    problem.q_init = np.copy(q_nom)
    problem.v_init = np.zeros(nv)
    problem.Qq = 0.1 * np.eye(nq)
    problem.Qv = 0.01 * np.eye(nv)
    problem.R = 0.1 * np.eye(nv)
    problem.Qf_q = 10.0 * np.eye(nq)
    problem.Qf_v = 1.0 * np.eye(nv)

    problem.q_nom = [np.copy(q_nom) for i in range(problem.num_steps + 1)]
    problem.v_nom = [np.zeros(nv) for i in range(problem.num_steps + 1)]

    # Set the solver parameters
    params = SolverParameters()
    params.max_iterations = 200
    params.scaling = True
    params.equality_constraints = True
    params.Delta0 = 1e1
    params.Delta_max = 1e5
    params.num_threads = 4
    params.contact_stiffness = 5000
    params.dissipation_velocity = 0.1
    params.smoothing_factor = 0.01
    params.friction_coefficient = 0.5
    params.stiction_velocity = 0.1
    params.verbose = True

    # Specify an initial guess
    q_guess = [np.copy(q_nom) for i in range(problem.num_steps + 1)]

    # Solve the optimization problem
    optimizer = TrajectoryOptimizer(diagram, plant, problem, params)
    solution = TrajectoryOptimizerSolution()
    stats = TrajectoryOptimizerStats()
    optimizer.Solve(q_guess, solution, stats)
    solve_time = np.sum(stats.iteration_times)
    print("Solve time: ", solve_time)

    # Visualize the solution
    meshcat = StartMeshcat()
    visualize_trajectory(solution.q, 0.05, model_file, meshcat)
    time.sleep(10)  # Wait for the visualizer to catch up
