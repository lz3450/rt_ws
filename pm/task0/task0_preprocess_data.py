#!/usr/bin/env python
"""
Robot Data Preprocessing Script

This script preprocesses robot joint data from CSV files for use with TCN models.
It performs the following operations:
1. Loads robot data from CSV
2. Converts timestamps from sec/nanosec to unified timestamp
3. Creates target values from command data
4. Computes joint position errors
5. Cleans and formats the data for machine learning

Usage:
    python task0_preprocess_data.py <input_csv_path> [output_csv_path]
"""

import argparse
import numpy as np
import pandas as pd
import sys
from pathlib import Path


def preprocess_data(input_path, output_path="task0_data.csv", verbose=True):
    """
    Preprocess robot data from CSV file.

    Args:
        input_path (str): Path to input CSV file
        output_path (str, optional): Path to save processed data. If None, returns DataFrame
        verbose (bool): Whether to print progress information

    Returns:
        pandas.DataFrame: Processed data if output_path is None
    """
    if verbose:
        print(f"Loading data from: {input_path}")

    # Load the data
    df = pd.read_csv(input_path)

    if verbose:
        print(f"Loaded data shape: {df.shape}")
        print(f"Columns: {list(df.columns)}")

    # Drop finger joint columns if they exist
    finger_columns = [
        "finger_joint1_pos",
        "finger_joint2_pos",
        "finger_joint1_eff",
        "finger_joint2_eff",
        "finger_joint1_vel",
        "finger_joint2_vel",
    ]
    if verbose:
        print(f"Dropping finger joint columns: {finger_columns}")
    df.drop(columns=finger_columns, inplace=True)

    # Convert timestamps from sec/nanosec to unified timestamp
    if verbose:
        print("Converting timestamps...")
    df["ts"] = df["sec"].astype("float64") + df["nanosec"].astype("float64") * 1e-9

    # Drop original timestamp columns
    df = df.drop(columns=["sec", "nanosec"])

    # Create target values from command data
    if verbose:
        print("Creating target values from command data...")

    # For each row, set 'joint{j}_target' to the most recent 'joint{j}_position' where t=='c' up to that row
    last_command = {f"joint{j + 1}_pos": np.nan for j in range(7)}
    targets = {f"joint{j + 1}_target": [] for j in range(7)}

    for i, row in df.iterrows():
        if row["t"] == "c":
            for j in range(7):
                last_command[f"joint{j + 1}_pos"] = row[f"joint{j + 1}_pos"]
        for j in range(7):
            targets[f"joint{j + 1}_target"].append(last_command[f"joint{j + 1}_pos"])

    # Drop the command/state indicator column
    df.drop(columns=["t"], inplace=True)

    # Add target columns to dataframe
    for j in range(7):
        df[f"joint{j + 1}_target"] = targets[f"joint{j + 1}_target"]

    # Drop rows with NaN values (before first command)
    if verbose:
        print(f"Dropping NaN values... Shape before: {df.shape}")
    df.dropna(inplace=True)
    if verbose:
        print(f"Shape after dropping NaN: {df.shape}")

    # Reset index
    df.reset_index(drop=True, inplace=True)

    # Calculate time differences
    if verbose:
        print("Calculating time differences...")
    df["dt"] = df["ts"].diff()

    # Drop timestamp column (keeping only dt)
    df.drop(columns=["ts"], inplace=True)

    # Drop first row with NaN dt
    df.dropna(inplace=True)

    # Reset index again
    df.reset_index(drop=True, inplace=True)

    # Calculate joint errors
    if verbose:
        print("Calculating joint position errors...")
    for j in range(7):
        df[f"joint{j + 1}_err"] = df[f"joint{j + 1}_target"] - df[f"joint{j + 1}_pos"]

    # Drop individual target and position columns, keeping only errors
    columns_to_drop = []
    for j in range(7):
        columns_to_drop.extend([f"joint{j + 1}_target", f"joint{j + 1}_pos"])
    df.drop(columns=columns_to_drop, inplace=True)

    if verbose:
        print(f"Final processed data shape: {df.shape}")
        print(f"Final columns: {list(df.columns)}")

    # Save or return data
    if output_path:
        if verbose:
            print(f"Saving processed data to: {output_path}")
        df.to_csv(output_path, index=False)


def main():
    """Main function for command line interface."""
    parser = argparse.ArgumentParser(
        description="Preprocess robot joint data for TCN training",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python task0_preprocess_data.py data/robot_data.csv
    python task0_preprocess_data.py data/robot_data.csv --output processed_data.csv
    python task0_preprocess_data.py data/robot_data.csv -o processed_data.csv --quiet
        """,
    )

    parser.add_argument("input_path", help="Path to input CSV file containing robot data")

    parser.add_argument(
        "-o",
        "--output",
        dest="output_path",
        default="task0_data.csv",
        help="Path to save processed data (if not specified, prints summary only)",
    )

    parser.add_argument("-q", "--quiet", action="store_true", help="Suppress progress output")

    args = parser.parse_args()

    # Check if input file exists
    if not Path(args.input_path).exists():
        print(f"Error: Input file '{args.input_path}' does not exist.")
        sys.exit(1)

    # Process the data
    preprocess_data(args.input_path, args.output_path, verbose=not args.quiet)


if __name__ == "__main__":
    main()
