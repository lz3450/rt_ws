#!/usr/bin/env python

"""
Robot State Prediction Function

This script implements the exact function requested: given current torques,
positions, and velocities, predict future positions and velocities every 1ms
for 10 steps (10ms total).



"""

import numpy as np
from urdf_parser import URDFParser
from robot_dynamics import RobotDynamics


def predict_robot_future(
    robot_dynamics: RobotDynamics, torques: np.ndarray, positions: np.ndarray, velocities: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """
    Predict future robot states given current torques, positions, and velocities.

    This function assumes constant torques and integrates the robot dynamics
    to predict future positions and velocities every 1ms for 10 steps.

    Args:
        robot_dynamics: The robot dynamics model
        torques: Current joint torques [n_joints] (assumed constant)
        positions: Current joint positions [n_joints]
        velocities: Current joint velocities [n_joints]

    Returns:
        tuple containing:
        - future_positions: Array of shape [10, n_joints] with positions at t=1ms, 2ms, ..., 10ms
        - future_velocities: Array of shape [10, n_joints] with velocities at t=1ms, 2ms, ..., 10ms
    """

    dt = 0.001  # 1ms time step
    n_steps = 10  # Predict for 10 steps
    n_joints = len(positions)

    # Initialize output arrays
    future_positions = np.zeros((n_steps, n_joints))
    future_velocities = np.zeros((n_steps, n_joints))

    # Current state
    q = positions.copy()
    q_dot = velocities.copy()

    # Integration loop using 4th-order Runge-Kutta for accuracy
    for step in range(n_steps):
        # RK4 integration step
        q, q_dot = rk4_step(robot_dynamics, q, q_dot, torques, dt)

        # Store results
        future_positions[step] = q
        future_velocities[step] = q_dot

    return future_positions, future_velocities


def rk4_step(
    robot: RobotDynamics, q: np.ndarray, q_dot: np.ndarray, tau: np.ndarray, dt: float
) -> tuple[np.ndarray, np.ndarray]:
    """
    Perform one step of 4th-order Runge-Kutta integration.

    The state is [q, q_dot] and the derivative is [q_dot, q_ddot]
    where q_ddot comes from forward dynamics.

    Args:
        robot: Robot dynamics model
        q: Current joint positions
        q_dot: Current joint velocities
        tau: Joint torques
        dt: Time step

    Returns:
        tuple of (q_new, q_dot_new)
    """

    # Helper function to get acceleration
    def get_acceleration(q_curr, q_dot_curr):
        q_ddot = robot.forward_dynamics(q_curr, q_dot_curr, tau)
        return robot._from_tensor(q_ddot)

    # k1 = f(t, x)
    q_ddot_k1 = get_acceleration(q, q_dot)
    q_k1 = q_dot
    q_dot_k1 = q_ddot_k1

    # k2 = f(t + dt/2, x + k1*dt/2)
    q_mid1 = q + q_k1 * dt / 2
    q_dot_mid1 = q_dot + q_dot_k1 * dt / 2
    q_ddot_k2 = get_acceleration(q_mid1, q_dot_mid1)
    q_k2 = q_dot_mid1
    q_dot_k2 = q_ddot_k2

    # k3 = f(t + dt/2, x + k2*dt/2)
    q_mid2 = q + q_k2 * dt / 2
    q_dot_mid2 = q_dot + q_dot_k2 * dt / 2
    q_ddot_k3 = get_acceleration(q_mid2, q_dot_mid2)
    q_k3 = q_dot_mid2
    q_dot_k3 = q_ddot_k3

    # k4 = f(t + dt, x + k3*dt)
    q_end = q + q_k3 * dt
    q_dot_end = q_dot + q_dot_k3 * dt
    q_ddot_k4 = get_acceleration(q_end, q_dot_end)
    q_k4 = q_dot_end
    q_dot_k4 = q_ddot_k4

    # Final update: x_new = x + (k1 + 2*k2 + 2*k3 + k4)*dt/6
    q_new = q + (q_k1 + 2 * q_k2 + 2 * q_k3 + q_k4) * dt / 6
    q_dot_new = q_dot + (q_dot_k1 + 2 * q_dot_k2 + 2 * q_dot_k3 + q_dot_k4) * dt / 6

    return q_new, q_dot_new


def test_prediction_function():
    """Test the prediction function with the Panda robot"""

    print("Testing the robot state prediction function")
    print("=" * 50)

    # Load robot
    urdf_path = "/home/kzl/Projects/RoboGuard/rg_ws/pm/panda_arm.urdf"
    parser = URDFParser(urdf_path)
    robot = RobotDynamics(parser, use_pytorch=False)

    print(f"✓ Loaded {parser.robot_name} robot with {robot.n_joints} DOF")

    # Test case 1: From rest with small torques
    print("\nTest case 1: Small torques from rest")
    print("-" * 30)

    torques = np.array([0.1, 0.2, -0.1, 0.15, -0.05, 0.08, 0.03])
    positions = np.zeros(7)
    velocities = np.zeros(7)

    print(f"Input torques:    {torques}")
    print(f"Input positions:  {positions}")
    print(f"Input velocities: {velocities}")

    # Predict future states
    future_pos, future_vel = predict_robot_future(robot, torques, positions, velocities)

    print("\nPredictions every 1ms for 10ms:")
    print("Time(ms) | Position changes | Velocity")
    print("-" * 60)
    for i in range(10):
        t_ms = i + 1
        pos_change = np.linalg.norm(future_pos[i] - positions)
        vel_norm = np.linalg.norm(future_vel[i])
        print(f"{t_ms:4d}     | {pos_change:12.6f}     | {vel_norm:8.4f}")

    print("\nFinal state at t=10ms:")
    print(f"Positions:  {future_pos[-1]}")
    print(f"Velocities: {future_vel[-1]}")

    # Test case 2: With initial motion and larger torques
    print("\n\nTest case 2: With initial motion")
    print("-" * 30)

    torques2 = np.array([1.0, -0.8, 1.5, -0.6, 0.9, -0.4, 0.5])
    positions2 = np.array([0.1, -0.1, 0.2, -0.3, 0.05, 0.2, 0.0])
    velocities2 = np.array([0.1, 0.05, -0.1, 0.02, 0.03, -0.02, 0.04])

    print(f"Input torques:    {torques2}")
    print(f"Input positions:  {positions2}")
    print(f"Input velocities: {velocities2}")

    future_pos2, future_vel2 = predict_robot_future(robot, torques2, positions2, velocities2)

    print("\nFinal state at t=10ms:")
    print(f"Positions:  {future_pos2[-1]}")
    print(f"Velocities: {future_vel2[-1]}")
    print(f"Position change: {future_pos2[-1] - positions2}")
    print(f"Velocity change: {future_vel2[-1] - velocities2}")

    # Verify the function output format
    print("\n\nFunction output verification:")
    print("-" * 30)
    print(f"future_positions shape: {future_pos.shape}")
    print(f"future_velocities shape: {future_vel.shape}")
    print(f"Output contains {future_pos.shape[0]} time steps")
    print(f"Each time step has {future_pos.shape[1]} joint values")
    print("Time step interval: 1ms")
    print(f"Total prediction horizon: {future_pos.shape[0]}ms")

    return robot, future_pos, future_vel


def example_usage():
    """Show example usage of the prediction function"""

    print("\n\n" + "=" * 60)
    print("EXAMPLE USAGE")
    print("=" * 60)

    print("""
# Load robot model
urdf_path = "robot.urdf"
parser = URDFParser(urdf_path)
robot = RobotDynamics(parser)

# Define current state
current_torques = np.array([0.1, 0.2, -0.1, 0.15, -0.05, 0.08, 0.03])
current_positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
current_velocities = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# Predict future states
future_positions, future_velocities = predict_robot_future(
    robot, current_torques, current_positions, current_velocities
)

# Access predictions
print("Position at t=1ms:", future_positions[0])   # First time step
print("Position at t=5ms:", future_positions[4])   # Fifth time step
print("Position at t=10ms:", future_positions[9])  # Last time step

print("Velocity at t=1ms:", future_velocities[0])  # First time step
print("Velocity at t=10ms:", future_velocities[9]) # Last time step
""")


if __name__ == "__main__":
    try:
        robot, future_pos, future_vel = test_prediction_function()
        example_usage()

        print("\n" + "=" * 60)
        print("✓ PREDICTION FUNCTION READY FOR USE!")
        print("=" * 60)

    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()
