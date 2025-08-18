#!/usr/bin/env python

"""
Fast Robot State Prediction

This module provides a fast implementation of robot state prediction
that uses simplified dynamics (without Coriolis terms) for real-time performance.

Function: predict_robot_future_fast()
- Inputs: torques, positions, velocities
- Outputs: future_positions, future_velocities (every 1ms for 10 steps)
- Uses: Mass matrix + gravity only (no Coriolis) for speed



"""

import numpy as np
from robot_dynamics import RobotDynamics


class FastRobotDynamics(RobotDynamics):
    """Fast robot dynamics with simplified Coriolis computation"""

    def compute_coriolis_matrix(self, q, q_dot):
        """Simplified Coriolis matrix (zeros for speed)"""
        return self._to_tensor(np.zeros((self.n_joints, self.n_joints)))


def predict_robot_future_fast(robot_dynamics: RobotDynamics,
                             torques: np.ndarray,
                             positions: np.ndarray,
                             velocities: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Fast prediction of future robot states using simplified dynamics.

    This function predicts future joint positions and velocities every 1ms
    for 10 steps, assuming constant torques. Uses simplified dynamics
    (Mass + Gravity only) for computational efficiency.

    Args:
        robot_dynamics: Robot dynamics model
        torques: Current joint torques [n_joints] - assumed constant
        positions: Current joint positions [n_joints]
        velocities: Current joint velocities [n_joints]

    Returns:
        tuple containing:
        - future_positions: Shape [10, n_joints] - positions at t=1ms, 2ms, ..., 10ms
        - future_velocities: Shape [10, n_joints] - velocities at t=1ms, 2ms, ..., 10ms
    """

    # Create a fast dynamics model
    fast_robot = FastRobotDynamics(robot_dynamics.parser, robot_dynamics.use_pytorch)

    # Fixed parameters
    dt = 0.001  # 1ms time step
    n_steps = 10  # 10 steps total
    n_joints = len(positions)

    # Initialize output arrays
    future_positions = np.zeros((n_steps, n_joints))
    future_velocities = np.zeros((n_steps, n_joints))

    # Current state
    q = positions.copy()
    q_dot = velocities.copy()

    # Time integration using simplified Euler method for speed
    for step in range(n_steps):
        # Compute acceleration using simplified dynamics
        q_ddot = fast_robot._from_tensor(fast_robot.forward_dynamics(q, q_dot, torques))

        # Euler integration (fast but less accurate)
        q_dot = q_dot + q_ddot * dt
        q = q + q_dot * dt

        # Store results
        future_positions[step] = q.copy()
        future_velocities[step] = q_dot.copy()

    return future_positions, future_velocities


def predict_robot_future_basic(mass_matrix: np.ndarray,
                              gravity_vector: np.ndarray,
                              torques: np.ndarray,
                              positions: np.ndarray,
                              velocities: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Very basic prediction using pre-computed mass matrix and gravity vector.

    This assumes the mass matrix and gravity vector are constant over the
    prediction horizon (valid for small time windows and small motions).

    Args:
        mass_matrix: Pre-computed mass matrix M(q) [n_joints, n_joints]
        gravity_vector: Pre-computed gravity vector G(q) [n_joints]
        torques: Constant joint torques [n_joints]
        positions: Initial joint positions [n_joints]
        velocities: Initial joint velocities [n_joints]

    Returns:
        tuple of (future_positions, future_velocities) arrays
    """

    dt = 0.001  # 1ms
    n_steps = 10
    n_joints = len(positions)

    # Initialize outputs
    future_positions = np.zeros((n_steps, n_joints))
    future_velocities = np.zeros((n_steps, n_joints))

    # Current state
    q = positions.copy()
    q_dot = velocities.copy()

    # Pre-compute inverse mass matrix
    M_inv = np.linalg.inv(mass_matrix)

    # Integration loop
    for step in range(n_steps):
        # Simple dynamics: q_ddot = M^(-1) * (tau - G)
        # (Ignoring Coriolis for speed)
        q_ddot = M_inv @ (torques - gravity_vector)

        # Euler integration
        q_dot = q_dot + q_ddot * dt
        q = q + q_dot * dt

        # Store results
        future_positions[step] = q
        future_velocities[step] = q_dot

    return future_positions, future_velocities


# Test and demonstration
if __name__ == "__main__":
    from urdf_parser import URDFParser

    print("Fast Robot State Prediction Test")
    print("=" * 40)

    # Load robot
    urdf_path = "/home/kzl/Projects/RoboGuard/rg_ws/pm/panda_arm.urdf"
    parser = URDFParser(urdf_path)
    robot = RobotDynamics(parser, use_pytorch=False)

    print(f"✓ Loaded {parser.robot_name} ({robot.n_joints} DOF)")

    # Test inputs
    test_torques = np.array([0.1, 0.2, -0.1, 0.15, -0.05, 0.08, 0.03])
    test_positions = np.zeros(7)
    test_velocities = np.zeros(7)

    print("\nTest Case: Fast Prediction")
    print(f"Torques:    {test_torques}")
    print(f"Positions:  {test_positions}")
    print(f"Velocities: {test_velocities}")

    # Method 1: Fast prediction with simplified dynamics
    print("\nRunning fast prediction...")
    future_pos_fast, future_vel_fast = predict_robot_future_fast(
        robot, test_torques, test_positions, test_velocities
    )

    print(f"✓ Fast prediction completed")
    print(f"Output shapes: {future_pos_fast.shape}, {future_vel_fast.shape}")

    # Method 2: Very basic prediction with constant matrices
    print("\nRunning basic prediction (constant matrices)...")

    # Pre-compute matrices at initial configuration
    M = robot._from_tensor(robot.compute_mass_matrix(test_positions))
    G = robot._from_tensor(robot.compute_gravity_vector(test_positions))

    future_pos_basic, future_vel_basic = predict_robot_future_basic(
        M, G, test_torques, test_positions, test_velocities
    )

    print(f"✓ Basic prediction completed")

    # Compare results
    print("\nComparison of methods:")
    print("Time | Fast method pos | Basic method pos | Difference")
    print("-" * 55)
    for i in range(10):
        t_ms = i + 1
        pos_fast = np.linalg.norm(future_pos_fast[i])
        pos_basic = np.linalg.norm(future_pos_basic[i])
        diff = abs(pos_fast - pos_basic)
        print(f"{t_ms:2d}ms | {pos_fast:13.6f} | {pos_basic:14.6f} | {diff:.2e}")

    print(f"\nFinal states at t=10ms:")
    print(f"Fast method  - Position: {future_pos_fast[-1]}")
    print(f"Fast method  - Velocity: {future_vel_fast[-1]}")
    print(f"Basic method - Position: {future_pos_basic[-1]}")
    print(f"Basic method - Velocity: {future_vel_basic[-1]}")

    # Demonstrate the requested function interface
    print("\n" + "=" * 50)
    print("REQUESTED FUNCTION INTERFACE")
    print("=" * 50)

    print("""
The function you requested is: predict_robot_future_fast()

Usage:
    future_positions, future_velocities = predict_robot_future_fast(
        robot_dynamics, torques, positions, velocities
    )

Where:
- Input torques, positions, velocities are current state arrays
- Output future_positions[i] = state at time (i+1) ms
- Output future_velocities[i] = velocities at time (i+1) ms
- i ranges from 0 to 9 (for 1ms to 10ms predictions)

Example access:
- future_positions[0] = position at t=1ms
- future_positions[4] = position at t=5ms
- future_positions[9] = position at t=10ms
""")

    print("✓ Function ready for use!")
    print("✓ Fast execution using simplified dynamics")
    print("✓ 1ms time steps for 10 steps as requested")
