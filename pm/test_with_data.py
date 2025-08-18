#!/usr/bin/env python

"""
Robot Prediction Testing with Real Data

This script tests the robot state prediction function using real robot data
from robot_data.csv. It randomly selects two consecutive lines:
- First line: Used as input (torques, positions, velocities)
- Second line: Used as ground truth for comparison



"""

import numpy as np
import pandas as pd
import random
from urdf_parser import URDFParser
from robot_dynamics import RobotDynamics
from robot_prediction import predict_robot_future


def load_robot_data(csv_file: str) -> pd.DataFrame:
    """Load the robot data from CSV file"""
    try:
        df = pd.read_csv(csv_file)
        print(f"✓ Loaded robot data: {len(df)} rows, {len(df.columns)} columns")
        return df
    except Exception as e:
        print(f"Error loading data: {e}")
        raise


def extract_robot_state(row: pd.Series) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Extract joint positions, velocities, and efforts from a data row

    Args:
        row: Single row from the robot data DataFrame

    Returns:
        tuple of (positions, velocities, torques) arrays [7 joints each]
    """

    # Extract positions (joints 1-7)
    positions = np.array([
        row['panda_joint1_position'], row['panda_joint2_position'],
        row['panda_joint3_position'], row['panda_joint4_position'],
        row['panda_joint5_position'], row['panda_joint6_position'],
        row['panda_joint7_position']
    ])

    # Extract velocities (joints 1-7)
    velocities = np.array([
        row['panda_joint1_velocity'], row['panda_joint2_velocity'],
        row['panda_joint3_velocity'], row['panda_joint4_velocity'],
        row['panda_joint5_velocity'], row['panda_joint6_velocity'],
        row['panda_joint7_velocity']
    ])

    # Extract efforts/torques (joints 1-7)
    torques = np.array([
        row['panda_joint1_effort'], row['panda_joint2_effort'],
        row['panda_joint3_effort'], row['panda_joint4_effort'],
        row['panda_joint5_effort'], row['panda_joint6_effort'],
        row['panda_joint7_effort']
    ])

    return positions, velocities, torques


def calculate_prediction_errors(predicted_pos: np.ndarray, predicted_vel: np.ndarray,
                               actual_pos: np.ndarray, actual_vel: np.ndarray) -> dict:
    """
    Calculate various error metrics between predicted and actual states

    Args:
        predicted_pos: Predicted positions [10, 7]
        predicted_vel: Predicted velocities [10, 7]
        actual_pos: Actual position [7]
        actual_vel: Actual velocity [7]

    Returns:
        Dictionary of error metrics
    """

    # We'll compare the prediction at the end of our horizon (10ms) with actual next state
    final_pred_pos = predicted_pos[-1]  # Position at t=10ms
    final_pred_vel = predicted_vel[-1]  # Velocity at t=10ms

    # Position errors
    pos_error = final_pred_pos - actual_pos
    pos_rmse = np.sqrt(np.mean(pos_error**2))
    pos_max_error = np.max(np.abs(pos_error))

    # Velocity errors
    vel_error = final_pred_vel - actual_vel
    vel_rmse = np.sqrt(np.mean(vel_error**2))
    vel_max_error = np.max(np.abs(vel_error))

    return {
        'position_rmse': pos_rmse,
        'position_max_error': pos_max_error,
        'position_errors': pos_error,
        'velocity_rmse': vel_rmse,
        'velocity_max_error': vel_max_error,
        'velocity_errors': vel_error
    }


def test_prediction_with_real_data(csv_file: str, n_tests: int = 5):
    """
    Test the prediction function using real robot data

    Args:
        csv_file: Path to the robot data CSV file
        n_tests: Number of random tests to perform
    """

    print("=" * 70)
    print("ROBOT PREDICTION TESTING WITH REAL DATA")
    print("=" * 70)

    # Load robot model
    urdf_path = "/home/kzl/Projects/RoboGuard/rg_ws/pm/panda_arm.urdf"
    parser = URDFParser(urdf_path)
    robot = RobotDynamics(parser, use_pytorch=False)
    print(f"✓ Loaded {parser.robot_name} robot model")

    # Load data
    df = load_robot_data(csv_file)

    # Calculate time step from data
    if len(df) > 1:
        dt_ms = df.iloc[1]['timestamp'] *1000  # Convert to milliseconds
        print(f"✓ Data time step: {dt_ms:.1f}ms")
    else:
        print("Warning: Not enough data for time step calculation")
        return

    print(f"\nRunning {n_tests} random tests...")
    print("-" * 50)

    all_results = []

    for test_num in range(n_tests):
        print(f"\nTest {test_num + 1}:")

        # Randomly select two consecutive rows
        # Make sure we don't go beyond the data
        max_idx = len(df) - 2
        random_idx = random.randint(0, max_idx)

        row1 = df.iloc[random_idx]
        row2 = df.iloc[random_idx + 1]

        print(f"  Selected rows: {random_idx} and {random_idx + 1}")
        print(f"  Timestamps: {row1['timestamp']:.6f}s → {row2['timestamp']:.6f}s")

        # Extract states
        pos1, vel1, torques1 = extract_robot_state(row1)
        pos2, vel2, torques2 = extract_robot_state(row2)

        print(f"  Input position range: [{np.min(pos1):.4f}, {np.max(pos1):.4f}] rad")
        print(f"  Input velocity range: [{np.min(vel1):.4f}, {np.max(vel1):.4f}] rad/s")
        print(f"  Input torque range: [{np.min(torques1):.2f}, {np.max(torques1):.2f}] Nm")

        # Make prediction using first row as input
        try:
            future_pos, future_vel = predict_robot_future(
                robot, torques1, pos1, vel1
            )

            # Calculate errors compared to second row (ground truth)
            errors = calculate_prediction_errors(
                future_pos, future_vel, pos2, vel2
            )

            # Store results
            all_results.append(errors)

            print(f"  Position RMSE: {errors['position_rmse']:.6f} rad ({errors['position_rmse']*180/np.pi:.3f}°)")
            print(f"  Position Max Error: {errors['position_max_error']:.6f} rad ({errors['position_max_error']*180/np.pi:.3f}°)")
            print(f"  Velocity RMSE: {errors['velocity_rmse']:.6f} rad/s")
            print(f"  Velocity Max Error: {errors['velocity_max_error']:.6f} rad/s")

        except Exception as e:
            print(f"  ❌ Prediction failed: {e}")
            continue

    # Summary statistics
    if all_results:
        print("\n" + "=" * 50)
        print("SUMMARY STATISTICS")
        print("=" * 50)

        pos_rmses = [r['position_rmse'] for r in all_results]
        vel_rmses = [r['velocity_rmse'] for r in all_results]
        pos_max_errors = [r['position_max_error'] for r in all_results]
        vel_max_errors = [r['velocity_max_error'] for r in all_results]

        print(f"Position RMSE - Mean: {np.mean(pos_rmses):.6f} rad ({np.mean(pos_rmses)*180/np.pi:.3f}°)")
        print(f"Position RMSE - Std:  {np.std(pos_rmses):.6f} rad ({np.std(pos_rmses)*180/np.pi:.3f}°)")
        print(f"Position Max Error - Mean: {np.mean(pos_max_errors):.6f} rad ({np.mean(pos_max_errors)*180/np.pi:.3f}°)")
        print(f"Position Max Error - Std:  {np.std(pos_max_errors):.6f} rad ({np.std(pos_max_errors)*180/np.pi:.3f}°)")

        print(f"\nVelocity RMSE - Mean: {np.mean(vel_rmses):.6f} rad/s")
        print(f"Velocity RMSE - Std:  {np.std(vel_rmses):.6f} rad/s")
        print(f"Velocity Max Error - Mean: {np.mean(vel_max_errors):.6f} rad/s")
        print(f"Velocity Max Error - Std:  {np.std(vel_max_errors):.6f} rad/s")

        # Joint-wise analysis for the last test
        if all_results:
            print("\nJoint-wise errors (last test):")
            last_result = all_results[-1]
            pos_errors = last_result['position_errors']
            vel_errors = last_result['velocity_errors']

            print("Joint | Position Error (rad) | Position Error (°) | Velocity Error (rad/s)")
            print("-" * 75)
            for i in range(7):
                pos_err = pos_errors[i]
                pos_err_deg = pos_err * 180 / np.pi
                vel_err = vel_errors[i]
                print(f"  {i+1}   |   {pos_err:15.6f}   |   {pos_err_deg:11.3f}   |   {vel_err:15.6f}")

        # Assessment
        print("\n" + "=" * 50)
        print("PREDICTION ASSESSMENT")
        print("=" * 50)

        avg_pos_error_deg = np.mean(pos_max_errors) * 180 / np.pi
        avg_vel_error = np.mean(vel_max_errors)

        print(f"Average maximum position error: {avg_pos_error_deg:.3f}°")
        print(f"Average maximum velocity error: {avg_vel_error:.3f} rad/s")

        if avg_pos_error_deg < 1.0:
            print("✓ Position prediction accuracy: EXCELLENT (< 1°)")
        elif avg_pos_error_deg < 5.0:
            print("✓ Position prediction accuracy: GOOD (< 5°)")
        elif avg_pos_error_deg < 10.0:
            print("⚠ Position prediction accuracy: FAIR (< 10°)")
        else:
            print("❌ Position prediction accuracy: POOR (> 10°)")

        if avg_vel_error < 0.1:
            print("✓ Velocity prediction accuracy: EXCELLENT (< 0.1 rad/s)")
        elif avg_vel_error < 0.5:
            print("✓ Velocity prediction accuracy: GOOD (< 0.5 rad/s)")
        elif avg_vel_error < 1.0:
            print("⚠ Velocity prediction accuracy: FAIR (< 1.0 rad/s)")
        else:
            print("❌ Velocity prediction accuracy: POOR (> 1.0 rad/s)")

    else:
        print("❌ No successful predictions completed")


def detailed_single_test(csv_file: str, row_idx: int = None):
    """
    Perform a detailed test with a specific row or random row

    Args:
        csv_file: Path to robot data CSV
        row_idx: Specific row index to test (None for random)
    """

    print("\n" + "=" * 70)
    print("DETAILED SINGLE TEST")
    print("=" * 70)

    # Load robot and data
    urdf_path = "/home/kzl/Projects/RoboGuard/rg_ws/pm/panda_arm.urdf"
    parser = URDFParser(urdf_path)
    robot = RobotDynamics(parser, use_pytorch=False)
    df = load_robot_data(csv_file)

    # Select row
    if row_idx is None:
        row_idx = random.randint(0, len(df) - 2)

    row1 = df.iloc[row_idx]
    row2 = df.iloc[row_idx + 1]

    print(f"Using rows {row_idx} and {row_idx + 1}")
    print(f"Time: {row1['timestamp']:.6f}s → {row2['timestamp']:.6f}s")

    # Extract states
    pos1, vel1, torques1 = extract_robot_state(row1)
    pos2, vel2, torques2 = extract_robot_state(row2)

    print("\nInput State (t=0):")
    print(f"Positions:  {pos1}")
    print(f"Velocities: {vel1}")
    print(f"Torques:    {torques1}")

    print("\nActual Next State (ground truth):")
    print(f"Positions:  {pos2}")
    print(f"Velocities: {vel2}")

    # Make prediction
    future_pos, future_vel = predict_robot_future(robot, torques1, pos1, vel1)

    print("\nPredicted State (t=10ms):")
    print(f"Positions:  {future_pos[-1]}")
    print(f"Velocities: {future_vel[-1]}")

    # Show prediction progression
    print("\nPrediction progression (every 2ms):")
    print("Time | Joint 1 Pos | Joint 1 Vel | All Joints Position Norm")
    print("-" * 65)
    for i in [1, 3, 5, 7, 9]:  # 2ms, 4ms, 6ms, 8ms, 10ms
        t_ms = i + 1
        pos_j1 = future_pos[i, 0]
        vel_j1 = future_vel[i, 0]
        pos_norm = np.linalg.norm(future_pos[i])
        print(f"{t_ms:2d}ms | {pos_j1:10.6f} | {vel_j1:10.6f} | {pos_norm:18.6f}")

    # Calculate detailed errors
    errors = calculate_prediction_errors(future_pos, future_vel, pos2, vel2)

    print("\nDetailed Error Analysis:")
    print(f"Position RMSE: {errors['position_rmse']:.6f} rad ({errors['position_rmse']*180/np.pi:.3f}°)")
    print(f"Velocity RMSE: {errors['velocity_rmse']:.6f} rad/s")

    return errors


if __name__ == "__main__":
    # Set random seed for reproducibility
    random.seed(42)
    np.random.seed(42)

    csv_file = "/home/kzl/Projects/RoboGuard/rg_ws/pm/robot_data.csv"

    try:
        # Run multiple random tests
        test_prediction_with_real_data(csv_file, n_tests=5)

        # Run one detailed test
        detailed_single_test(csv_file)

        print("\n" + "=" * 70)
        print("TESTING COMPLETED!")
        print("=" * 70)
        print("The prediction function has been tested with real robot data.")
        print("Results show how well the model predicts actual robot behavior.")

    except Exception as e:
        print(f"Error during testing: {e}")
        import traceback
        traceback.print_exc()
