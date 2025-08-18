#!/usr/bin/env python

"""
Robot State Prediction - Final Implementation

This script implements the exact function requested:
Given current torques, positions, and velocities, predict future positions
and velocities every 1ms for 10 steps (10ms total).

Function: predict_robot_future()
- Assumes constant torques throughout prediction
- Uses simplified dynamics for computational efficiency
- Returns future states at t=1ms, 2ms, ..., 10ms



"""

import numpy as np
from robot_dynamics import RobotDynamics


class SimplifiedRobotDynamics(RobotDynamics):
    """Simplified robot dynamics for fast prediction"""

    def compute_coriolis_matrix(self, q, q_dot):
        """Use zero Coriolis matrix for computational speed"""
        return self._to_tensor(np.zeros((self.n_joints, self.n_joints)))


def predict_robot_future(robot_dynamics: RobotDynamics,
                         torques: np.ndarray,
                         positions: np.ndarray,
                         velocities: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Predict future robot states given current torques, positions, and velocities.

    This function integrates the robot dynamics forward in time assuming constant
    torques to predict future joint positions and velocities every 1ms for 10 steps.

    Args:
        robot_dynamics: The robot dynamics model (RobotDynamics instance)
        torques: Current joint torques [n_joints] - assumed to remain constant
        positions: Current joint positions [n_joints]
        velocities: Current joint velocities [n_joints]

    Returns:
        tuple containing:
        - future_positions: Array of shape [10, n_joints]
          Each row i contains positions at time t = (i+1) ms
        - future_velocities: Array of shape [10, n_joints]
          Each row i contains velocities at time t = (i+1) ms

    Example:
        future_pos, future_vel = predict_robot_future(robot, tau, q, q_dot)
        pos_at_1ms = future_pos[0]   # Position at t=1ms
        pos_at_5ms = future_pos[4]   # Position at t=5ms
        pos_at_10ms = future_pos[9]  # Position at t=10ms
    """

    # Create simplified dynamics model for speed
    simplified_robot = SimplifiedRobotDynamics(
        robot_dynamics.parser, robot_dynamics.use_pytorch
    )

    # Fixed parameters as requested
    dt = 0.001  # 1ms time step
    n_steps = 10  # 10 prediction steps
    n_joints = len(positions)

    # Initialize output arrays
    future_positions = np.zeros((n_steps, n_joints))
    future_velocities = np.zeros((n_steps, n_joints))

    # Current state (will be updated during integration)
    q = positions.copy()
    q_dot = velocities.copy()

    # Time integration loop
    for step in range(n_steps):
        # Compute acceleration using forward dynamics
        # τ = M(q)q̈ + G(q) => q̈ = M⁻¹(τ - G(q))
        q_ddot = simplified_robot._from_tensor(
            simplified_robot.forward_dynamics(q, q_dot, torques)
        )

        # Euler integration (simple and fast)
        q_dot = q_dot + q_ddot * dt  # Update velocity
        q = q + q_dot * dt           # Update position

        # Store predicted state
        future_positions[step] = q.copy()
        future_velocities[step] = q_dot.copy()

    return future_positions, future_velocities


# Test and demonstration
def test_prediction():
    """Test the prediction function"""

    from urdf_parser import URDFParser

    print("Robot State Prediction Test")
    print("=" * 35)

    # Load robot model
    urdf_path = "/home/kzl/Projects/RoboGuard/rg_ws/pm/panda_arm.urdf"
    parser = URDFParser(urdf_path)
    robot = RobotDynamics(parser, use_pytorch=False)

    print(f"✓ Robot: {parser.robot_name} ({robot.n_joints} DOF)")

    # Define test inputs
    current_torques = np.array([0.1, 0.2, -0.1, 0.15, -0.05, 0.08, 0.03])
    current_positions = np.zeros(7)  # Start from home position
    current_velocities = np.zeros(7)  # Start from rest

    print(f"\nInputs:")
    print(f"  Torques:    {current_torques}")
    print(f"  Positions:  {current_positions}")
    print(f"  Velocities: {current_velocities}")

    # Call the prediction function
    future_pos, future_vel = predict_robot_future(
        robot, current_torques, current_positions, current_velocities
    )

    print(f"\nOutput:")
    print(f"  future_positions shape: {future_pos.shape}")
    print(f"  future_velocities shape: {future_vel.shape}")

    print(f"\nPredicted states every 1ms:")
    print("Time | Joint 1 Position | Joint 1 Velocity | All Joints Norm")
    print("-" * 65)

    for i in range(10):
        t_ms = i + 1
        pos_j1 = future_pos[i, 0]
        vel_j1 = future_vel[i, 0]
        pos_norm = np.linalg.norm(future_pos[i])
        print(f"{t_ms:2d}ms | {pos_j1:15.6f} | {vel_j1:15.6f} | {pos_norm:13.6f}")

    print(f"\nFinal state at t=10ms:")
    print(f"  Positions:  {future_pos[-1]}")
    print(f"  Velocities: {future_vel[-1]}")

    # Verify physical consistency
    print(f"\nPhysical validation:")
    initial_speed = np.linalg.norm(current_velocities)
    final_speed = np.linalg.norm(future_vel[-1])
    print(f"  Initial speed: {initial_speed:.6f} rad/s")
    print(f"  Final speed:   {final_speed:.6f} rad/s")
    print(f"  Speed change:  {final_speed - initial_speed:.6f} rad/s")

    max_position_change = np.max(np.abs(future_pos[-1] - current_positions))
    print(f"  Max joint displacement: {max_position_change:.6f} rad")
    print(f"  Max joint displacement: {max_position_change * 180/np.pi:.3f} degrees")

    return future_pos, future_vel


if __name__ == "__main__":
    try:
        future_positions, future_velocities = test_prediction()

        print(f"\n" + "=" * 50)
        print("FUNCTION READY FOR USE!")
        print("=" * 50)

        print("""
Usage:
    future_positions, future_velocities = predict_robot_future(
        robot_dynamics, torques, positions, velocities
    )

Where:
- torques, positions, velocities: current robot state arrays
- future_positions[i]: joint positions at time t = (i+1) ms
- future_velocities[i]: joint velocities at time t = (i+1) ms
- i ranges from 0 to 9 (1ms to 10ms predictions)

Access examples:
- future_positions[0] → state at t=1ms
- future_positions[4] → state at t=5ms
- future_positions[9] → state at t=10ms
""")

        print("✓ Function implemented successfully")
        print("✓ Time step: 1ms")
        print("✓ Prediction horizon: 10 steps (10ms)")
        print("✓ Assumes constant torques")
        print("✓ Uses simplified dynamics for speed")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
