##
#
# Helper utilities for performing MPC in simulation with pyidto.
#
##

import numpy as np
import time
from pydrake.all import (
    LeafSystem,
    BasicVector,
    EventStatus,
    PiecewisePolynomial,
    Value,
)

from pyidto import TrajectoryOptimizerSolution, TrajectoryOptimizerStats


class StoredTrajectory:
    """
    A simple class for storing a polynomial representation of a trajectory
    that we might get from pyidto.
    """
    start_time = None  # The time at which the trajectory starts
    q = None           # A PiecewisePolynomial representing the generalized coordinates
    v = None           # A PiecewisePolynomial representing the generalized velocities
    tau = None         # A PiecewisePolynomial representing the generalized forces
    dt = None


class Interpolator(LeafSystem):
    """
    A simple Drake system that interpolates a StoredTrajectory to provide the
    actuated state reference x(t) and input u(t) at a given time t. This is
    useful for passing a reference trajectory to a low-level controller.
    """

    def __init__(self, Bq, Bv):
        """
        Construct the interpolator system, which takes StoredTrajectory as input
        and produces the state and input at a given time.

                             ------------------
                             |                | --->  x(t)
            trajectory --->  |  Interpolator  |
                             |                | --->  u(t)
                             ------------------

        Args:
            Bq: Actuated DoF selection matrix for generalized coordinates
            Bv: Actuator selection matrix for generalized velocities and forces
        """
        LeafSystem.__init__(self)

        # Check that the actuated selection matrices are the right size
        num_actuators = Bq.shape[0]
        assert Bv.shape[0] == num_actuators

        self.Bq = Bq
        self.Bv = Bv

        # Declare the input and output ports
        trajectory_input_port = self.DeclareAbstractInputPort("trajectory",
                                                              Value(StoredTrajectory()))
        state_output_port = self.DeclareVectorOutputPort("state",
                                                         BasicVector(
                                                             2 * num_actuators),
                                                         self.SendState)
        control_output_port = self.DeclareVectorOutputPort("control",
                                                           BasicVector(
                                                               num_actuators),
                                                           self.SendControl)

    def SendState(self, context, output):
        """
        Send the state at the current time.
        """
        trajectory = self.EvalAbstractInput(context, 0).get_value()
        t = context.get_time() - trajectory.start_time
        ind = np.min((int(np.floor(t/trajectory.dt)), trajectory.q.shape[1]-1))
        if ind == trajectory.q.shape[1]-1:
            q = self.Bq @ trajectory.q[:, ind]
            v = self.Bv @ trajectory.v[:, ind]
        else:
            lamb = (t-trajectory.dt*ind)/trajectory.dt
            q = self.Bq @ ((1-lamb)*trajectory.q[:, ind] + lamb*trajectory.q[:, ind+1]) 
            v = self.Bv @ ((1-lamb)*trajectory.v[:, ind] + lamb*trajectory.v[:, ind+1]) 
        #q = self.Bq @ trajectory.q.value(t)
        #v = self.Bv @ trajectory.v.value(t)
        output.SetFromVector(np.concatenate((q, v)))

    def SendControl(self, context, output):
        """
        Send the control input at the current time.
        """
        trajectory = self.EvalAbstractInput(context, 0).get_value()
        t = (context.get_time() - trajectory.start_time)
        ind = np.min((int(np.floor(t/trajectory.dt)), trajectory.q.shape[1]-1))
        if ind == trajectory.tau.shape[1]:
            u = trajectory.tau[:, ind]
        else:
            lamb = (t-trajectory.dt*ind)/trajectory.dt
            u = ((1-lamb)*trajectory.tau[:, ind] + lamb*trajectory.tau[:, ind+1]) 
        #u = trajectory.tau.value(context.get_time() -
        #                                   trajectory.start_time)
        output.SetFromVector(u)


class ModelPredictiveController(LeafSystem):
    """
    A Drake system that implements an MPC controller.
    """

    def __init__(self, optimizer, q_guess, nq, nv, mpc_rate):
        """
        Construct the MPC controller system, which takes the current state as
        input and sends an optimial StoredTrajectory as output. 

                         -------------------------------
                         |                             |
            state  --->  |  ModelPredictiveController  |  --->  trajectory
                         |                             |
                         -------------------------------

        Args:
            optimizer: A TrajectoryOptimizer object that can solve the MPC
                       problem.
            q_guess: An initial guess for the trajectory optimization problem.
            nq: The number of generalized coordinates
            nv: The number of generalized velocities
            mpc_rate: The rate at which the MPC problem is to be solved (Hz)
        """
        LeafSystem.__init__(self)

        self.optimizer = optimizer
        self.nq = nq

        # Allocate a warm-start
        self.q_guess = q_guess
        self.warm_start = self.optimizer.CreateWarmStart(self.q_guess)

        # Specify the timestep we'll use to discretize the trajectory
        self.time_step = self.optimizer.time_step()
        self.num_steps = self.optimizer.num_steps()

        # Solve the optimization problem to get the initial trajectory
        solution = TrajectoryOptimizerSolution()
        stats = TrajectoryOptimizerStats()
        self.optimizer.SolveFromWarmStart(self.warm_start, solution, stats)

        # Declare an abstract-valued state that will hold the optimal trajectory
        state = self.StoreOptimizerSolution(solution, 0.0)
        self.stored_trajectory = self.DeclareAbstractState(Value(state))

        # Define a periodic update event that will trigger the optimizer to
        # resolve the MPC problem.
        self.DeclarePeriodicUnrestrictedUpdateEvent(
            1. / mpc_rate, 0, self.UpdateAbstractState)

        # Declare the input and output ports
        self.state_input_port = self.DeclareVectorInputPort(
            "state", BasicVector(nq + nv))
        self.trajectory_output_port = self.DeclareStateOutputPort(
            "optimal_trajectory", self.stored_trajectory)

    def StoreOptimizerSolution(self, solution, start_time):
        """
        Store a solution to the optimization problem in a StoredTrajectory object.

        Args:
            solution: A TrajectoryOptimizerSolution object
            start_time: The time at which the trajectory starts

        Returns:
            A StoredTrajectory object containing an interpolation of the solution.
        """
        # Create numpy arrays with knot points for iterpolation of the solution
        # along the actuated DoFs
        time_steps = np.linspace(
            0, self.time_step * self.num_steps, self.num_steps + 1)
        q_knots = np.array(solution.q).T
        v_knots = np.array(solution.v).T
        tau_knots = solution.tau
        tau_knots.append(solution.tau[-1])  # Repeat the last control input
        tau_knots = np.array(tau_knots).T

        # Create the StoredTrajectory object
        trajectory = StoredTrajectory()
        trajectory.start_time = start_time
        trajectory.q = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            time_steps, q_knots)
        trajectory.v = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            time_steps, v_knots)
        trajectory.tau = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            time_steps, tau_knots)

        return trajectory

    def UpdateAbstractState(self, context, state):
        """
        Resolve the MPC problem and store the optimal trajectory in the abstract
        state.
        """
        print(f"Resolving at t={context.get_time()}")

        # Get the current state
        x0 = self.state_input_port.Eval(context)
        q0 = x0[:self.nq]
        v0 = x0[self.nq:]
        self.optimizer.ResetInitialConditions(q0, v0)

        # Shift the warm-start based on the stored interpolation and time elapsed
        last_trajectory = state.get_abstract_state(0).get_value()
        self.q_guess[0] = q0
        start_time = context.get_time() - last_trajectory.start_time
        for i in range(1, self.num_steps + 1):
            t = start_time + i * self.time_step
            self.q_guess[i] = last_trajectory.q.value(t).flatten()
        self.warm_start.set_q(self.q_guess)

        # Shift the nominal trajectory as needed
        self.UpdateNominalTrajectory(context)

        # Solve the optimization problem
        solution = TrajectoryOptimizerSolution()
        stats = TrajectoryOptimizerStats()
        self.optimizer.SolveFromWarmStart(self.warm_start, solution, stats)

        # Store the solution in the abstract state
        state.get_mutable_abstract_state(0).SetFrom(
            Value(self.StoreOptimizerSolution(solution, context.get_time())))

        return EventStatus.Succeeded()

    def UpdateNominalTrajectory(self, context):
        """
        Shift the nominal trajectory to account for the current state.
        This may or may not be useful depending on the specifics of the task.
        """
        pass

class OpenLoopPositionController(LeafSystem):
    """
    A Drake system that implements an Open Loop controller.
    """

    def __init__(self, optimizer, q_guess, nq, nv, nu):
        """
        Construct the Open Loop controller system, which takes no input and sends
        a StoredTrajectory as output, based on Inverse Dynamics. 

                         -------------------------------
                         |                             |
                         |     OpenLoopController      |  --->  trajectory
                         |                             |
                         -------------------------------

        Args:
            optimizer: A TrajectoryOptimizer object that can provide ID
            q_guess: An initial guess for the ID.
            nq: The number of generalized coordinates
            nv: The number of generalized velocities
        """
        LeafSystem.__init__(self)

        self.optimizer = optimizer
        self.nq = nq
        self.nv = nv
        self.nu = nu

        # Allocate a warm-start
        self.q_guess = q_guess

        # Specify the timestep we'll use to discretize the trajectory
        self.time_step = self.optimizer.time_step()
        self.num_steps = self.optimizer.num_steps()

        # Solve the inverse dynamics get the initial trajectory
        self.state = self.StoreOptimizerSolution(self.q_guess, 0.0)
        self.stored_trajectory = self.DeclareAbstractState(Value(self.state))

        # Declare the output port
        self.trajectory_output_port = self.DeclareStateOutputPort(
            "optimal_trajectory", self.stored_trajectory)
        
    def ResetOpenLoopTrajectory(self, context, new_q):

        # Store the solution in the abstract state
        context.get_mutable_abstract_state(self.stored_trajectory).SetFrom(
            Value(self.StoreOptimizerSolution(new_q, 0.0)))
        
        return EventStatus.Succeeded()


    def StoreOptimizerSolution(self, q_guess, start_time):
        """
        Store a solution to the inverse dynamics problem in a StoredTrajectory object.

        Args:
            q_guess: A nominal trajectory
            start_time: The time at which the trajectory starts

        Returns:
            A StoredTrajectory object containing an interpolation of the solution.
        """
        t1 = time.time()
        # Run inverse dynamics
        #init_state = self.optimizer.CreateState()
        t12 = time.time()
        #init_state.set_q(q_guess)
        q_state = q_guess
        t13 = time.time()
        #v_state = self.optimizer.EvalV(init_state)
        #tau_state = self.optimizer.EvalTau(init_state)
        t2 = time.time()

        # Create numpy arrays with knot points for iterpolation of the solution
        # along the actuated DoFs
        time_steps = np.linspace(
            0, self.time_step * self.num_steps, self.num_steps + 1)
        q_knots = np.array(q_state).T
        v_knots = np.gradient(q_knots, self.optimizer.time_step(), axis=1)#np.array(v_state).T
        #tau_knots = tau_state
        #tau_knots.append(tau_state[-1])  # Repeat the last control input
        v_knots_norm = np.linalg.norm(v_knots, axis=0)
        tau_knots = np.zeros((self.nu, v_knots.shape[1]))#np.array(tau_knots).T

        t3 = time.time()
        # Create the StoredTrajectory object
        trajectory = StoredTrajectory()
        trajectory.start_time = start_time
        trajectory.q = q_knots
        trajectory.v = v_knots
        trajectory.tau = tau_knots
        trajectory.dt = self.time_step
        # trajectory.q = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
        #     time_steps, q_knots)
        # trajectory.v = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
        #     time_steps, v_knots)
        # trajectory.tau = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
        #     time_steps, tau_knots)
        t4 = time.time()

        # print("Part1: ", (t2-t1)/(t4-t1)) #82%
        # print("Part1_1: ", (t12 - t1)/(t2-t1)) #98% of Part1
        # print("Part1_2: ", (t13 - t12)/(t2-t1))
        # print("Part1_3: ", (t2 - t13)/(t2-t1))
        # print("Part2: ", (t3-t2)/(t4-t1)) #3%
        # print("Part3: ", (t4-t3)/(t4-t1)) #15%
        # print("Controller timer: ", t4-t1)

        return trajectory

    def UpdateNominalTrajectory(self, context):
        """
        Shift the nominal trajectory to account for the current state.
        This may or may not be useful depending on the specifics of the task.
        """
        pass

class OpenLoopTorqueController(LeafSystem):
    """
    A Drake system that implements an Open Loop controller.
    """

    def __init__(self, optimizer, input_commands, init_state, manipulator_delta, nq):
        """
        Construct the Open Loop controller system, which takes no input and sends
        a StoredTrajectory as output, which just provides the requested torques. 

                         -------------------------------
                         |                             |
                         |     OpenLoopController      |  --->  trajectory
                         |                             |
                         -------------------------------

        Args:
            optimizer: A TrajectoryOptimizer object that can provide ID
            input_commands: The open loop input commands to send
        """
        LeafSystem.__init__(self)

        self.optimizer = optimizer
        self.nq = nq

        # Allocate a warm-start
        self.input_commands = input_commands
        self.init_state = init_state
        self.manipulator_delta = manipulator_delta

        # Specify the timestep we'll use to discretize the trajectory
        self.time_step = self.optimizer.time_step()
        self.num_steps = self.optimizer.num_steps()

        self.state = self.StoreOptimizerSolution(self.input_commands, self.init_state, self.manipulator_delta, 0.0)
        self.stored_trajectory = self.DeclareAbstractState(Value(self.state))

        # Declare the output port
        self.trajectory_output_port = self.DeclareStateOutputPort(
            "optimal_trajectory", self.stored_trajectory)
        
    def ResetOpenLoopTrajectory(self, context,  input_commands, init_state, manipulator_delta):

        # Store the solution in the abstract state
        context.get_mutable_abstract_state(self.stored_trajectory).SetFrom(
            Value(self.StoreOptimizerSolution(input_commands, init_state, manipulator_delta, 0.0)))
        
        return EventStatus.Succeeded()


    def StoreOptimizerSolution(self, input_commands, init_state, manipulator_delta, start_time):
        """

        Args:
            input_commands: The open loop input commands
            start_time: The time at which the trajectory starts

        Returns:
            A StoredTrajectory object containing an interpolation of the solution.
        """

        q_state = input_commands


        # Create numpy arrays with knot points for iterpolation of the solution
        # along the actuated DoFs
        time_steps = np.linspace(
            0, self.time_step * self.num_steps, self.num_steps + 1)
        q_knots = np.linspace(init_state, init_state + 2000*manipulator_delta, self.num_steps+1).T
        v_knots = np.gradient(q_knots, axis=1)
        tau_knots = 0*np.array(input_commands).T
        zero_knots = np.zeros((self.nq, self.num_steps+1))

        # Create the StoredTrajectory object
        trajectory = StoredTrajectory()
        trajectory.start_time = start_time
        
        trajectory.q = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            time_steps, q_knots)
        trajectory.v = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            time_steps, v_knots)
        trajectory.tau = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            time_steps, tau_knots)

        return trajectory

    def UpdateNominalTrajectory(self, context):
        """
        Shift the nominal trajectory to account for the current state.
        This may or may not be useful depending on the specifics of the task.
        """
        pass

