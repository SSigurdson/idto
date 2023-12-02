#!/usr/bin/env python

##
#
# An example of planar walking using the harpy model.
#
# IDTO (and pyidto) must be compiled with bazel first:
#
#  bazel build //...
#
##

# Add pyidto to the python path
import os
import sys
sys.path.insert(-1, os.getcwd() + "/bazel-bin/")

import numpy as np
import time
from copy import deepcopy

from pydrake.all import (StartMeshcat, DiagramBuilder,
        AddMultibodyPlantSceneGraph, AddDefaultVisualization, Parser)

from pyidto.trajectory_optimizer import TrajectoryOptimizer
from pyidto.trajectory_optimizer import TrajectoryOptimizer
from pyidto.problem_definition import ProblemDefinition
from pyidto.solver_parameters import SolverParameters
from pyidto.trajectory_optimizer_solution import TrajectoryOptimizerSolution
from pyidto.trajectory_optimizer_stats import TrajectoryOptimizerStats

def define_optimization_problem():
    """
    Specify a nominal trajectory and cost function.
    """
    velocity_target = 1.0  # Desired forward velocity

    q_start = np.array([0.0,   # base horizontal position
                        0.5,   # base vertical position
                        0.0,   # base orientation
                       -0.45,  # right hip
                        1.15,  # right knee
                       -0.45,  # left hip
                        1.15]) # left knee
    q_end = deepcopy(q_start)
    q_end[0] = velocity_target

    problem = ProblemDefinition()
    problem.num_steps = 40
    problem.q_init = q_start
    problem.v_init = np.zeros(7)
    problem.Qq = np.diag([1, 1, 1, 1, 1, 1, 1])
    problem.Qv = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    problem.R = np.diag([1e5, 1e5, 1e5,
                         0.1, 0.1, 0.1, 0.1])
    problem.Qf_q = 5 * np.eye(7)
    problem.Qf_v = 0.5 * np.eye(7)

    q_nom = []   # Can't use list comprehension here because of Eigen conversion
    v_nom = []
    for i in range(problem.num_steps + 1):
        sigma = i / problem.num_steps
        q_nom.append((1 - sigma) * q_start + sigma * q_end)
        v_nom.append(np.array([velocity_target, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    problem.q_nom = q_nom
    problem.v_nom = v_nom

    return problem

def define_solver_parameters():
    """
    Create a set of solver parameters, including contact modeling parameters.
    """
    params = SolverParameters()

    # Trust region solver parameters
    params.max_iterations = 200
    params.scaling = False
    params.equality_constraints = False
    params.Delta0 = 1e1
    params.Delta_max = 1e5
    params.num_threads = 4

    # Contact modeling parameters
    params.contact_stiffness = 5e3
    params.dissipation_velocity = 0.1
    params.smoothing_factor = 0.01
    params.friction_coefficient = 0.5
    params.stiction_velocity = 0.05

    params.verbose = True

    return params

def visualize_trajectory(q, time_step, model_file, meshcat=None):
    """
    Display the given trajectory (list of configurations) on meshcat
    """
    # Create a simple Drake diagram with a plant model
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
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
    # Start up meshcat (for viewing the result)
    meshcat = StartMeshcat()

    # Relative path to the model file that we'll use
    model_file = "./examples/models/harpy_planar.urdf"

    # Specify a cost function and target trajectory
    problem = define_optimization_problem()
    time_step = 0.05

    # Visualize the nominal trajectory (optional)
    # visualize_trajectory(problem.q_nom, time_step, model_file, meshcat)

    # Specify solver parameters, including contact modeling parameters
    params = define_solver_parameters()

    # Specify an initial guess
    q_guess = deepcopy(problem.q_nom)

    # Create the optimizer object
    opt = TrajectoryOptimizer(model_file, problem, params, time_step)

    # Allocate some structs that will hold the solution
    solution = TrajectoryOptimizerSolution()
    stats = TrajectoryOptimizerStats()

    # Solve the optimization problem
    opt.Solve(q_guess, solution, stats)

    solve_time = np.sum(stats.iteration_times)
    print(f"Solved in {solve_time:.4f} seconds")
   
    # Play back the solution on meshcat
    visualize_trajectory(solution.q, time_step, model_file, meshcat)

