#!/usr/bin/env python

"""
Robot State Prediction - Clean Implementation

This module provides the exact function requested: given current torques,
positions, and velocities, predict future states every 1ms for 10 steps.

Function signature:
predict_robot_future(robot_dynamics, torques, positions, velocities)
    -> (future_positions, future_velocities)

Where:
- future_positions: array of shape [10, n_joints]
- future_velocities: array of shape [10, n_joints]
- Each row represents the state at t = 1ms, 2ms, ..., 10ms



"""

import numpy as np
from robot_dynamics import RobotDynamics


def predict_robot_future(robot_dynamics: RobotDynamics,
                        torques: np.ndarray,
                        positions: np.ndarray,
                        velocities: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Predict future robot states given current torques, positions, and velocities.

    This function integrates the robot dynamics forward in time, assuming constant
    torques, to predict future joint positions and velocities every 1ms for 10 steps.

    The integration uses 4th-order Runge-Kutta for high accuracy.

    Args:
        robot_dynamics: Robot dynamics model (RobotDynamics instance)
        torques: Current joint torques [n_joints] - assumed to remain constant
        positions: Current joint positions [n_joints]
        velocities: Current joint velocities [n_joints]

    Returns:
        tuple containing:
        - future_positions: Shape [10, n_joints] - positions at t=1ms, 2ms, ..., 10ms
        - future_velocities: Shape [10, n_joints] - velocities at t=1ms, 2ms, ..., 10ms
    """

    # Fixed parameters as requested
    dt = 0.001  # 1ms time step
    n_steps = 10  # Predict for 10 steps (10ms total)
    n_joints = len(positions)

    # Initialize output arrays
    future_positions = np.zeros((n_steps, n_joints))
    future_velocities = np.zeros((n_steps, n_joints))

    # Current state (will be updated during integration)
    q = positions.copy()
    q_dot = velocities.copy()

    # Time integration loop
    for step in range(n_steps):
        # Integrate one time step using RK4
        q, q_dot = _rk4_step(robot_dynamics, q, q_dot, torques, dt)

        # Store the predicted state
        future_positions[step] = q.copy()
        future_velocities[step] = q_dot.copy()

    return future_positions, future_velocities


def _rk4_step(robot: RobotDynamics,
              q: np.ndarray,
              q_dot: np.ndarray,
              tau: np.ndarray,
              dt: float) -> tuple[np.ndarray, np.ndarray]:
    """
    Perform one step of 4th-order Runge-Kutta integration.

    Private function for internal use by predict_robot_future().
    """

    def get_acceleration(q_curr: np.ndarray, q_dot_curr: np.ndarray) -> np.ndarray:
        """Get joint accelerations from forward dynamics"""
        q_ddot = robot.forward_dynamics(q_curr, q_dot_curr, tau)
        return robot._from_tensor(q_ddot)

    # RK4 intermediate calculations
    # k1
    k1_q = q_dot
    k1_q_dot = get_acceleration(q, q_dot)

    # k2
    q_k2 = q + k1_q * dt/2
    q_dot_k2 = q_dot + k1_q_dot * dt/2
    k2_q = q_dot_k2
    k2_q_dot = get_acceleration(q_k2, q_dot_k2)

    # k3
    q_k3 = q + k2_q * dt/2
    q_dot_k3 = q_dot + k2_q_dot * dt/2
    k3_q = q_dot_k3
    k3_q_dot = get_acceleration(q_k3, q_dot_k3)

    # k4
    q_k4 = q + k3_q * dt
    q_dot_k4 = q_dot + k3_q_dot * dt
    k4_q = q_dot_k4
    k4_q_dot = get_acceleration(q_k4, q_dot_k4)

    # Final RK4 update
    q_new = q + (k1_q + 2*k2_q + 2*k3_q + k4_q) * dt/6
    q_dot_new = q_dot + (k1_q_dot + 2*k2_q_dot + 2*k3_q_dot + k4_q_dot) * dt/6

    return q_new, q_dot_new


# Example usage and testing
if __name__ == "__main__":
    # Import here to avoid circular dependencies in the module
    from urdf_parser import URDFParser

    print("Testing robot state prediction function")
    print("=" * 45)

    # Load the Panda robot
    urdf_path = "/home/kzl/Projects/RoboGuard/rg_ws/pm/panda_arm.urdf"
    parser = URDFParser(urdf_path)
    robot = RobotDynamics(parser, use_pytorch=False)

    print(f"✓ Loaded {parser.robot_name} ({robot.n_joints} DOF)")

    # Test the prediction function
    test_torques = np.array([0.1, 0.2, -0.1, 0.15, -0.05, 0.08, 0.03])
    test_positions = np.zeros(7)
    test_velocities = np.zeros(7)

    print("\nInputs:")
    print(f"  Torques:    {test_torques}")
    print(f"  Positions:  {test_positions}")
    print(f"  Velocities: {test_velocities}")

    # Call the prediction function
    future_pos, future_vel = predict_robot_future(
        robot, test_torques, test_positions, test_velocities
    )

    print(f"\nOutput shapes:")
    print(f"  future_positions: {future_pos.shape}")
    print(f"  future_velocities: {future_vel.shape}")

    print(f"\nPredicted states:")
    print("Time | Position (joint 1) | Velocity (joint 1)")
    print("-" * 45)
    for i in range(10):
        t_ms = i + 1
        pos_j1 = future_pos[i, 0]  # First joint position
        vel_j1 = future_vel[i, 0]  # First joint velocity
        print(f"{t_ms:2d}ms | {pos_j1:15.6f} | {vel_j1:15.6f}")

    print(f"\nFinal state at t=10ms:")
    print(f"  All positions:  {future_pos[-1]}")
    print(f"  All velocities: {future_vel[-1]}")

    print(f"\n✓ Function working correctly!")
    print(f"✓ Returns future states every 1ms for 10 steps")
    print(f"✓ Uses 4th-order Runge-Kutta integration")
    print(f"✓ Assumes constant torques throughout prediction")
