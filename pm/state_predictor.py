#!/usr/bin/env python

"""
Robot State Prediction using Forward Dynamics Integration

This script implements time integration to predict future robot states given
current torques, positions, and velocities. It uses forward dynamics to compute
accelerations and then integrates over time to predict future states.

The integration uses:
1. Forward dynamics: τ → q̈ (given current q, q̇)
2. Numerical integration: q̈ → q̇ → q over time



"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple
from urdf_parser import URDFParser
from robot_dynamics import RobotDynamics


class RobotStatePredictor:
    """
    A class for predicting future robot states using forward dynamics integration.
    """

    def __init__(self, robot_dynamics: RobotDynamics):
        """
        Initialize the state predictor.

        Args:
            robot_dynamics: The robot dynamics model
        """
        self.robot = robot_dynamics
        self.n_joints = robot_dynamics.n_joints

    def predict_future_states(
        self,
        tau: np.ndarray,
        q_current: np.ndarray,
        q_dot_current: np.ndarray,
        dt: float = 0.001,  # 1ms time step
        n_steps: int = 10,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Predict future robot states using forward dynamics integration.

        This function assumes constant torques and integrates the robot dynamics
        to predict future positions and velocities.

        Integration method: 4th-order Runge-Kutta (RK4) for accuracy

        Args:
            tau: Constant joint torques [n_joints]
            q_current: Current joint positions [n_joints]
            q_dot_current: Current joint velocities [n_joints]
            dt: Time step in seconds (default: 0.001s = 1ms)
            n_steps: Number of prediction steps (default: 10)

        Returns:
            Tuple of:
            - time_array: Time points [n_steps+1]
            - q_trajectory: Joint positions over time [n_steps+1, n_joints]
            - q_dot_trajectory: Joint velocities over time [n_steps+1, n_joints]
        """

        # Initialize arrays to store trajectory
        time_array = np.arange(n_steps + 1) * dt
        q_trajectory = np.zeros((n_steps + 1, self.n_joints))
        q_dot_trajectory = np.zeros((n_steps + 1, self.n_joints))

        # Set initial conditions
        q_trajectory[0] = q_current.copy()
        q_dot_trajectory[0] = q_dot_current.copy()

        print(f"Predicting robot states for {n_steps} steps with dt = {dt * 1000:.1f}ms")
        print(f"Initial position: {q_current}")
        print(f"Initial velocity: {q_dot_current}")
        print(f"Constant torques: {tau}")
        print("\nIntegrating forward dynamics...")

        # Current state
        q = q_current.copy()
        q_dot = q_dot_current.copy()

        # Integration loop using RK4
        for step in range(n_steps):
            # RK4 integration step
            q_new, q_dot_new = self._rk4_step(q, q_dot, tau, dt)

            # Update state
            q = q_new
            q_dot = q_dot_new

            # Store results
            q_trajectory[step + 1] = q
            q_dot_trajectory[step + 1] = q_dot

            # Print progress every few steps
            if (step + 1) % 5 == 0 or step == 0:
                print(f"  Step {step + 1:2d}: t = {time_array[step + 1] * 1000:4.1f}ms")

        print("✓ Integration completed successfully")
        return time_array, q_trajectory, q_dot_trajectory

    def _rk4_step(self, q: np.ndarray, q_dot: np.ndarray, tau: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Perform one step of 4th-order Runge-Kutta integration.

        The state vector is: x = [q, q_dot]
        The derivative is: dx/dt = [q_dot, q_ddot]

        Where q_ddot comes from forward dynamics: q_ddot = f(q, q_dot, tau)

        Args:
            q: Current joint positions
            q_dot: Current joint velocities
            tau: Joint torques
            dt: Time step

        Returns:
            Tuple of (q_new, q_dot_new)
        """

        # RK4 coefficients
        # k1 = f(t, x)
        q_ddot_k1 = self._compute_acceleration(q, q_dot, tau)
        q_k1 = q_dot
        q_dot_k1 = q_ddot_k1

        # k2 = f(t + dt/2, x + k1*dt/2)
        q_mid1 = q + q_k1 * dt / 2
        q_dot_mid1 = q_dot + q_dot_k1 * dt / 2
        q_ddot_k2 = self._compute_acceleration(q_mid1, q_dot_mid1, tau)
        q_k2 = q_dot_mid1
        q_dot_k2 = q_ddot_k2

        # k3 = f(t + dt/2, x + k2*dt/2)
        q_mid2 = q + q_k2 * dt / 2
        q_dot_mid2 = q_dot + q_dot_k2 * dt / 2
        q_ddot_k3 = self._compute_acceleration(q_mid2, q_dot_mid2, tau)
        q_k3 = q_dot_mid2
        q_dot_k3 = q_ddot_k3

        # k4 = f(t + dt, x + k3*dt)
        q_end = q + q_k3 * dt
        q_dot_end = q_dot + q_dot_k3 * dt
        q_ddot_k4 = self._compute_acceleration(q_end, q_dot_end, tau)
        q_k4 = q_dot_end
        q_dot_k4 = q_ddot_k4

        # Final update: x_new = x + (k1 + 2*k2 + 2*k3 + k4)*dt/6
        q_new = q + (q_k1 + 2 * q_k2 + 2 * q_k3 + q_k4) * dt / 6
        q_dot_new = q_dot + (q_dot_k1 + 2 * q_dot_k2 + 2 * q_dot_k3 + q_dot_k4) * dt / 6

        return q_new, q_dot_new

    def _compute_acceleration(self, q: np.ndarray, q_dot: np.ndarray, tau: np.ndarray) -> np.ndarray:
        """
        Compute joint accelerations using forward dynamics.

        Args:
            q: Joint positions
            q_dot: Joint velocities
            tau: Joint torques

        Returns:
            Joint accelerations q_ddot
        """
        q_ddot = self.robot.forward_dynamics(q, q_dot, tau)
        return self.robot._from_tensor(q_ddot)

    def predict_with_euler(
        self, tau: np.ndarray, q_current: np.ndarray, q_dot_current: np.ndarray, dt: float = 0.001, n_steps: int = 10
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Alternative prediction using simple Euler integration (less accurate but faster).

        Args:
            tau: Constant joint torques [n_joints]
            q_current: Current joint positions [n_joints]
            q_dot_current: Current joint velocities [n_joints]
            dt: Time step in seconds
            n_steps: Number of prediction steps

        Returns:
            Tuple of (time_array, q_trajectory, q_dot_trajectory)
        """

        # Initialize arrays
        time_array = np.arange(n_steps + 1) * dt
        q_trajectory = np.zeros((n_steps + 1, self.n_joints))
        q_dot_trajectory = np.zeros((n_steps + 1, self.n_joints))

        # Set initial conditions
        q_trajectory[0] = q_current.copy()
        q_dot_trajectory[0] = q_dot_current.copy()

        q = q_current.copy()
        q_dot = q_dot_current.copy()

        # Euler integration: much simpler but less accurate
        for step in range(n_steps):
            # Compute acceleration at current state
            q_ddot = self._compute_acceleration(q, q_dot, tau)

            # Euler update
            q_dot = q_dot + q_ddot * dt  # v_new = v_old + a * dt
            q = q + q_dot * dt  # x_new = x_old + v * dt

            # Store results
            q_trajectory[step + 1] = q
            q_dot_trajectory[step + 1] = q_dot

        return time_array, q_trajectory, q_dot_trajectory


def demo_state_prediction():
    """Demonstrate state prediction functionality"""

    print("=" * 70)
    print("ROBOT STATE PREDICTION DEMONSTRATION")
    print("=" * 70)

    # Initialize robot
    urdf_path = "/home/kzl/Projects/RoboGuard/rg_ws/pm/panda_arm.urdf"
    parser = URDFParser(urdf_path)
    robot = RobotDynamics(parser, use_pytorch=False)
    predictor = RobotStatePredictor(robot)

    print(f"✓ Loaded {parser.robot_name} robot with {robot.n_joints} DOF")

    # Test scenario 1: Small motion from rest
    print("\n" + "=" * 50)
    print("SCENARIO 1: Small torques from rest position")
    print("=" * 50)

    q_initial = np.zeros(robot.n_joints)  # Start at home position
    q_dot_initial = np.zeros(robot.n_joints)  # Start from rest
    tau_constant = np.array([0.1, 0.2, -0.1, 0.15, -0.05, 0.08, 0.03])  # Small torques

    # Predict using RK4
    t, q_traj, q_dot_traj = predictor.predict_future_states(
        tau_constant, q_initial, q_dot_initial, dt=0.001, n_steps=10
    )

    print("\nResults after 10ms:")
    print(f"Final positions:  {q_traj[-1]}")
    print(f"Final velocities: {q_dot_traj[-1]}")
    print(f"Position change:  {q_traj[-1] - q_traj[0]}")
    print(f"Max velocity:     {np.max(np.abs(q_dot_traj[-1])):.6f} rad/s")

    # Test scenario 2: Motion from non-zero initial states
    print("\n" + "=" * 50)
    print("SCENARIO 2: Larger torques with initial motion")
    print("=" * 50)

    q_initial_2 = np.array([0.1, -0.2, 0.3, -0.5, 0.1, 0.4, 0.0])
    q_dot_initial_2 = np.array([0.1, 0.05, -0.08, 0.02, 0.03, -0.01, 0.04])
    tau_constant_2 = np.array([2.0, -1.5, 3.0, -0.8, 1.2, -0.5, 0.7])

    t2, q_traj_2, q_dot_traj_2 = predictor.predict_future_states(
        tau_constant_2, q_initial_2, q_dot_initial_2, dt=0.001, n_steps=10
    )

    print("\nResults after 10ms:")
    print(f"Initial positions: {q_initial_2}")
    print(f"Final positions:   {q_traj_2[-1]}")
    print(f"Position change:   {q_traj_2[-1] - q_traj_2[0]}")
    print(f"Initial velocity:  {q_dot_initial_2}")
    print(f"Final velocities:  {q_dot_traj_2[-1]}")
    print(f"Velocity change:   {q_dot_traj_2[-1] - q_dot_traj_2[0]}")

    # Compare RK4 vs Euler
    print("\n" + "=" * 50)
    print("INTEGRATION METHOD COMPARISON")
    print("=" * 50)

    # Same initial conditions, compare methods
    t_euler, q_euler, q_dot_euler = predictor.predict_with_euler(
        tau_constant, q_initial, q_dot_initial, dt=0.001, n_steps=10
    )

    print("\nAfter 10ms - Position differences (RK4 vs Euler):")
    pos_diff = q_traj[-1] - q_euler[-1]
    print(f"Max difference: {np.max(np.abs(pos_diff)):.2e} rad")
    print(f"RMS difference: {np.sqrt(np.mean(pos_diff**2)):.2e} rad")

    print("\nAfter 10ms - Velocity differences (RK4 vs Euler):")
    vel_diff = q_dot_traj[-1] - q_dot_euler[-1]
    print(f"Max difference: {np.max(np.abs(vel_diff)):.2e} rad/s")
    print(f"RMS difference: {np.sqrt(np.mean(vel_diff**2)):.2e} rad/s")

    # Physical validation
    print("\n" + "=" * 50)
    print("PHYSICAL VALIDATION")
    print("=" * 50)

    # Check energy conservation (should increase due to applied torques)
    initial_energy = compute_total_energy(robot, q_initial, q_dot_initial)
    final_energy = compute_total_energy(robot, q_traj[-1], q_dot_traj[-1])

    print(f"Initial total energy: {initial_energy:.6f} J")
    print(f"Final total energy:   {final_energy:.6f} J")
    print(f"Energy change:        {final_energy - initial_energy:.6f} J")
    print(f"Work done by torques: {np.sum(tau_constant * (q_traj[-1] - q_traj[0])):.6f} J")

    # Check if motion is reasonable
    max_joint_speed = np.max(np.abs(q_dot_traj))
    print(f"Maximum joint speed:  {max_joint_speed:.4f} rad/s ({max_joint_speed * 180 / np.pi:.2f} deg/s)")

    # Plot results if matplotlib is available
    try:
        plot_prediction_results(t, q_traj, q_dot_traj, "RK4 Integration Results")
        plot_comparison(t, q_traj, q_euler, q_dot_traj, q_dot_euler)
    except ImportError:
        print("Matplotlib not available for plotting")

    return predictor, t, q_traj, q_dot_traj


def compute_total_energy(robot: RobotDynamics, q: np.ndarray, q_dot: np.ndarray) -> float:
    """Compute total mechanical energy (kinetic + potential)"""
    # Kinetic energy: T = 0.5 * q_dot^T * M(q) * q_dot
    M = robot._from_tensor(robot.compute_mass_matrix(q))
    T = 0.5 * q_dot.T @ M @ q_dot

    # Potential energy: V = -sum(m_i * g^T * p_i)
    # We'll use a reference height (zero potential at base)
    V = 0.0
    transformations = robot.forward_kinematics(q)

    for i, link_name in enumerate(robot.link_names):
        link_inertial = robot.parser.get_link_inertial(link_name)
        if link_inertial and link_inertial.mass > 0 and i < len(transformations):
            # Get link position
            T_link = transformations[i]
            com_world = T_link @ np.append(link_inertial.center_of_mass, 1)
            height = com_world[2]  # Z-coordinate
            V += link_inertial.mass * 9.81 * height

    return float(T + V)


def plot_prediction_results(t: np.ndarray, q_traj: np.ndarray, q_dot_traj: np.ndarray, title: str):
    """Plot the prediction results"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

    # Plot positions
    ax1.set_title(f"{title} - Joint Positions")
    for i in range(q_traj.shape[1]):
        ax1.plot(t * 1000, q_traj[:, i], "o-", label=f"Joint {i + 1}", markersize=3)
    ax1.set_xlabel("Time (ms)")
    ax1.set_ylabel("Position (rad)")
    ax1.legend()
    ax1.grid(True)

    # Plot velocities
    ax2.set_title(f"{title} - Joint Velocities")
    for i in range(q_dot_traj.shape[1]):
        ax2.plot(t * 1000, q_dot_traj[:, i], "s-", label=f"Joint {i + 1}", markersize=3)
    ax2.set_xlabel("Time (ms)")
    ax2.set_ylabel("Velocity (rad/s)")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig("/home/kzl/Projects/RoboGuard/rg_ws/pm/state_prediction.png", dpi=300, bbox_inches="tight")
    print("✓ Results saved to 'state_prediction.png'")


def plot_comparison(
    t: np.ndarray, q_rk4: np.ndarray, q_euler: np.ndarray, q_dot_rk4: np.ndarray, q_dot_euler: np.ndarray
):
    """Plot comparison between RK4 and Euler methods"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

    # Plot position differences
    ax1.set_title("Integration Method Comparison - Position Differences")
    for i in range(q_rk4.shape[1]):
        diff = q_rk4[:, i] - q_euler[:, i]
        ax1.plot(t * 1000, diff, "o-", label=f"Joint {i + 1}", markersize=3)
    ax1.set_xlabel("Time (ms)")
    ax1.set_ylabel("Position Difference (RK4 - Euler) [rad]")
    ax1.legend()
    ax1.grid(True)

    # Plot velocity differences
    ax2.set_title("Integration Method Comparison - Velocity Differences")
    for i in range(q_dot_rk4.shape[1]):
        diff = q_dot_rk4[:, i] - q_dot_euler[:, i]
        ax2.plot(t * 1000, diff, "s-", label=f"Joint {i + 1}", markersize=3)
    ax2.set_xlabel("Time (ms)")
    ax2.set_ylabel("Velocity Difference (RK4 - Euler) [rad/s]")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig("/home/kzl/Projects/RoboGuard/rg_ws/pm/integration_comparison.png", dpi=300, bbox_inches="tight")
    print("✓ Comparison saved to 'integration_comparison.png'")


if __name__ == "__main__":
    try:
        predictor, t, q_traj, q_dot_traj = demo_state_prediction()

        print(f"\n" + "=" * 70)
        print("PREDICTION FUNCTION READY FOR USE!")
        print("=" * 70)
        print("\nExample usage:")
        print("```python")
        print("predictor = RobotStatePredictor(robot_dynamics)")
        print("t, q_future, q_dot_future = predictor.predict_future_states(")
        print("    tau=torques, q_current=positions, q_dot_current=velocities,")
        print("    dt=0.001, n_steps=10)")
        print("```")

    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()
