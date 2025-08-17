#!/usr/bin/env python

"""
Robotic Arm Dynamics Model using Lagrangian Mechanics

This module implements a class to model robotic arm dynamics using the Lagrange equations.
The implementation uses the parsed URDF data and computes:
- Forward kinematics
- Mass matrix (inertia matrix)
- Coriolis and centrifugal forces
- Gravitational forces
- Forward and inverse dynamics

Author: Assistant
Date: August 16, 2025
"""

import numpy as np
import torch
from typing import List, Tuple, Union
from urdf_parser import URDFParser, JointInfo


class RobotDynamics:
    """
    A class to model robotic arm dynamics using Lagrangian mechanics.

    The Lagrange equations of motion for a robotic manipulator are:
    τ = M(q)q̈ + C(q,q̇)q̇ + G(q)

    Where:
    - τ: joint torques
    - M(q): mass/inertia matrix
    - C(q,q̇): Coriolis and centrifugal matrix
    - G(q): gravitational forces
    - q: joint positions
    - q̇: joint velocities
    - q̈: joint accelerations
    """

    def __init__(self, urdf_parser: URDFParser, use_pytorch: bool = False):
        """
        Initialize the robot dynamics model.

        Args:
            urdf_parser: Parsed URDF data
            use_pytorch: Whether to use PyTorch tensors (True) or NumPy arrays (False)
        """
        self.parser = urdf_parser
        self.use_pytorch = use_pytorch
        self.n_joints = urdf_parser.get_dof()
        self.joint_names = urdf_parser.get_joint_names()
        self.link_names = urdf_parser.get_link_names()

        # Physical constants
        self.gravity = np.array([0, 0, -9.81])  # Gravity vector in base frame

        # Cache for computational efficiency
        self._transformation_matrices = {}
        self._jacobians = {}

        print(f"Initialized robot dynamics model for {self.parser.robot_name}")
        print(f"DOF: {self.n_joints}, Using PyTorch: {use_pytorch}")

    def _to_tensor(self, array: np.ndarray) -> Union[np.ndarray, torch.Tensor]:
        """Convert numpy array to tensor if using PyTorch"""
        if self.use_pytorch:
            return torch.from_numpy(array).float()
        return array

    def _from_tensor(self, tensor: Union[np.ndarray, torch.Tensor]) -> np.ndarray:
        """Convert tensor back to numpy array"""
        if self.use_pytorch and isinstance(tensor, torch.Tensor):
            return tensor.detach().cpu().numpy()
        return tensor

    def forward_kinematics(self, q: Union[np.ndarray, torch.Tensor]) -> List[np.ndarray]:
        """
        Compute forward kinematics for all links.

        Args:
            q: Joint positions [n_joints]

        Returns:
            List of 4x4 transformation matrices from base to each link
        """
        q = self._from_tensor(q)

        transformations = []
        current_transform = np.eye(4)  # Start with identity (base frame)

        joint_idx = 0
        for i, link_name in enumerate(self.link_names):
            if i == 0:
                # Base link
                transformations.append(current_transform.copy())
                continue

            # Find the joint that connects to this link
            joint_name = None
            for jname in self.joint_names:
                joint_info = self.parser.get_joint_info(jname)
                if joint_info and joint_info.child_link == link_name:
                    joint_name = jname
                    break

            if joint_name is None:
                # Fixed connection or end-effector
                transformations.append(current_transform.copy())
                continue

            joint_info = self.parser.get_joint_info(joint_name)
            if joint_info is None:
                continue

            # Get joint transformation
            joint_transform = self._get_joint_transformation(joint_info, q[joint_idx])
            current_transform = current_transform @ joint_transform
            transformations.append(current_transform.copy())

            joint_idx += 1
            if joint_idx >= len(q):
                break

        return transformations

    def _get_joint_transformation(self, joint_info: JointInfo, q_i: float) -> np.ndarray:
        """
        Get the transformation matrix for a single joint.

        Args:
            joint_info: Joint information from URDF
            q_i: Joint angle (for revolute joints)

        Returns:
            4x4 transformation matrix
        """
        # Static transformation from URDF (parent to joint frame)
        T_static = self.parser.get_transformation_matrix(
            joint_info.origin_xyz, joint_info.origin_rpy
        )

        if joint_info.joint_type == 'revolute':
            # Dynamic transformation (rotation about joint axis)
            axis = joint_info.axis / np.linalg.norm(joint_info.axis)

            # Rodrigues' rotation formula for arbitrary axis
            cos_q = np.cos(q_i)
            sin_q = np.sin(q_i)

            # Skew-symmetric matrix of axis
            K = np.array([
                [0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0]
            ])

            # Rotation matrix using Rodrigues' formula
            R_joint = np.eye(3) + sin_q * K + (1 - cos_q) * (K @ K)

            T_joint = np.eye(4)
            T_joint[:3, :3] = R_joint

            return T_static @ T_joint

        else:
            # For fixed joints, just return static transformation
            return T_static

    def compute_jacobian(self, q: Union[np.ndarray, torch.Tensor], link_idx: int) -> np.ndarray:
        """
        Compute the geometric Jacobian for a specific link.

        Args:
            q: Joint positions [n_joints]
            link_idx: Index of the link in the kinematic chain

        Returns:
            6x(n_joints) Jacobian matrix [linear_velocity; angular_velocity]
        """
        q = self._from_tensor(q)

        # Get all transformation matrices
        transformations = self.forward_kinematics(q)

        if link_idx >= len(transformations):
            raise ValueError(f"Link index {link_idx} out of range")

        # Position of the end-effector (or target link)
        p_end = transformations[link_idx][:3, 3]

        # Initialize Jacobian
        J = np.zeros((6, self.n_joints))

        joint_idx = 0
        for i in range(min(link_idx, len(self.joint_names))):
            joint_name = self.joint_names[joint_idx] if joint_idx < len(self.joint_names) else None
            if joint_name is None:
                continue

            joint_info = self.parser.get_joint_info(joint_name)
            if joint_info is None or joint_info.joint_type != 'revolute':
                continue

            # Joint position and axis in world frame
            if i < len(transformations):
                T_joint = transformations[i]
                p_joint = T_joint[:3, 3]
                z_joint = T_joint[:3, :3] @ joint_info.axis
                z_joint = z_joint / np.linalg.norm(z_joint)

                # Linear velocity contribution
                J[:3, joint_idx] = np.cross(z_joint, p_end - p_joint)

                # Angular velocity contribution
                J[3:, joint_idx] = z_joint

            joint_idx += 1

        return J

    def compute_mass_matrix(self, q: Union[np.ndarray, torch.Tensor]) -> Union[np.ndarray, torch.Tensor]:
        """
        Compute the mass/inertia matrix M(q).

        Args:
            q: Joint positions [n_joints]

        Returns:
            Mass matrix M(q) [n_joints x n_joints]
        """
        q_np = self._from_tensor(q)

        # Initialize mass matrix
        M = np.zeros((self.n_joints, self.n_joints))

        # Get forward kinematics
        transformations = self.forward_kinematics(q_np)

        # For each link, compute its contribution to the mass matrix
        for i, link_name in enumerate(self.link_names):
            link_inertial = self.parser.get_link_inertial(link_name)
            if link_inertial is None or link_inertial.mass <= 0:
                continue

            # Get transformation to this link
            if i >= len(transformations):
                continue

            T_link = transformations[i]

            # Note: Center of mass transformation would be computed as:
            # com_local = np.append(link_inertial.center_of_mass, 1)
            # com_world = T_link @ com_local
            # But not needed for current mass matrix calculation

            # Transform inertia matrix to world frame
            R_link = T_link[:3, :3]
            I_world = R_link @ link_inertial.inertia_matrix @ R_link.T

            # Compute Jacobian for this link's center of mass
            J_v = self.compute_jacobian(q_np, i)[:3, :]  # Linear velocity Jacobian
            J_w = self.compute_jacobian(q_np, i)[3:, :]  # Angular velocity Jacobian

            # Contribution to mass matrix
            M += link_inertial.mass * (J_v.T @ J_v) + (J_w.T @ I_world @ J_w)

        return self._to_tensor(M)

    def compute_coriolis_matrix(self, q: Union[np.ndarray, torch.Tensor],
                               q_dot: Union[np.ndarray, torch.Tensor]) -> Union[np.ndarray, torch.Tensor]:
        """
        Compute the Coriolis and centrifugal matrix C(q,q̇).

        Args:
            q: Joint positions [n_joints]
            q_dot: Joint velocities [n_joints]

        Returns:
            Coriolis matrix C(q,q̇) [n_joints x n_joints]
        """
        q_np = self._from_tensor(q)
        q_dot_np = self._from_tensor(q_dot)

        # Compute Coriolis matrix using Christoffel symbols
        C = np.zeros((self.n_joints, self.n_joints))

        # Small perturbation for numerical differentiation
        epsilon = 1e-6

        # Note: Current mass matrix computation for reference (not used in Coriolis calculation)
        # M = self._from_tensor(self.compute_mass_matrix(q_np))

        for i in range(self.n_joints):
            for j in range(self.n_joints):
                for k in range(self.n_joints):
                    # Compute partial derivatives of M using finite differences
                    q_plus = q_np.copy()
                    q_plus[k] += epsilon
                    M_plus = self._from_tensor(self.compute_mass_matrix(q_plus))

                    q_minus = q_np.copy()
                    q_minus[k] -= epsilon
                    M_minus = self._from_tensor(self.compute_mass_matrix(q_minus))

                    # Partial derivative of M[i,j] with respect to q[k]
                    dM_ij_dqk = (M_plus[i, j] - M_minus[i, j]) / (2 * epsilon)

                    # Partial derivative of M[i,k] with respect to q[j]
                    q_plus = q_np.copy()
                    q_plus[j] += epsilon
                    M_plus = self._from_tensor(self.compute_mass_matrix(q_plus))

                    q_minus = q_np.copy()
                    q_minus[j] -= epsilon
                    M_minus = self._from_tensor(self.compute_mass_matrix(q_minus))

                    dM_ik_dqj = (M_plus[i, k] - M_minus[i, k]) / (2 * epsilon)

                    # Partial derivative of M[j,k] with respect to q[i]
                    q_plus = q_np.copy()
                    q_plus[i] += epsilon
                    M_plus = self._from_tensor(self.compute_mass_matrix(q_plus))

                    q_minus = q_np.copy()
                    q_minus[i] -= epsilon
                    M_minus = self._from_tensor(self.compute_mass_matrix(q_minus))

                    dM_jk_dqi = (M_plus[j, k] - M_minus[j, k]) / (2 * epsilon)

                    # Christoffel symbol
                    c_ijk = 0.5 * (dM_ij_dqk + dM_ik_dqj - dM_jk_dqi)

                    # Add contribution to Coriolis matrix
                    C[i, j] += c_ijk * q_dot_np[k]

        return self._to_tensor(C)

    def compute_gravity_vector(self, q: Union[np.ndarray, torch.Tensor]) -> Union[np.ndarray, torch.Tensor]:
        """
        Compute the gravitational force vector G(q).

        Args:
            q: Joint positions [n_joints]

        Returns:
            Gravity vector G(q) [n_joints]
        """
        q_np = self._from_tensor(q)

        # Initialize gravity vector
        G = np.zeros(self.n_joints)

        # Get forward kinematics
        transformations = self.forward_kinematics(q_np)

        # For each link, compute gravitational contribution
        for i, link_name in enumerate(self.link_names):
            link_inertial = self.parser.get_link_inertial(link_name)
            if link_inertial is None or link_inertial.mass <= 0:
                continue

            if i >= len(transformations):
                continue

            # Compute Jacobian for this link's center of mass
            J_v = self.compute_jacobian(q_np, i)[:3, :]  # Linear velocity Jacobian

            # Gravitational force contribution
            G += link_inertial.mass * (J_v.T @ self.gravity)

        return self._to_tensor(G)

    def forward_dynamics(self, q: Union[np.ndarray, torch.Tensor],
                        q_dot: Union[np.ndarray, torch.Tensor],
                        tau: Union[np.ndarray, torch.Tensor]) -> Union[np.ndarray, torch.Tensor]:
        """
        Compute forward dynamics: given joint torques, compute joint accelerations.

        Solves: q̈ = M⁻¹(τ - C*q̇ - G)

        Args:
            q: Joint positions [n_joints]
            q_dot: Joint velocities [n_joints]
            tau: Joint torques [n_joints]

        Returns:
            Joint accelerations q̈ [n_joints]
        """
        # Compute dynamics matrices
        M = self.compute_mass_matrix(q)
        C = self.compute_coriolis_matrix(q, q_dot)
        G = self.compute_gravity_vector(q)

        if self.use_pytorch:
            # PyTorch implementation
            M_inv = torch.linalg.inv(M)
            q_ddot = M_inv @ (tau - C @ q_dot - G)
        else:
            # NumPy implementation
            M_np = self._from_tensor(M)
            C_np = self._from_tensor(C)
            G_np = self._from_tensor(G)
            tau_np = self._from_tensor(tau)
            q_dot_np = self._from_tensor(q_dot)

            q_ddot = np.linalg.solve(M_np, tau_np - C_np @ q_dot_np - G_np)
            q_ddot = self._to_tensor(q_ddot)

        return q_ddot

    def inverse_dynamics(self, q: Union[np.ndarray, torch.Tensor],
                        q_dot: Union[np.ndarray, torch.Tensor],
                        q_ddot: Union[np.ndarray, torch.Tensor]) -> Union[np.ndarray, torch.Tensor]:
        """
        Compute inverse dynamics: given joint accelerations, compute required torques.

        Computes: τ = M*q̈ + C*q̇ + G

        Args:
            q: Joint positions [n_joints]
            q_dot: Joint velocities [n_joints]
            q_ddot: Joint accelerations [n_joints]

        Returns:
            Required joint torques τ [n_joints]
        """
        # Compute dynamics matrices
        M = self.compute_mass_matrix(q)
        C = self.compute_coriolis_matrix(q, q_dot)
        G = self.compute_gravity_vector(q)

        if self.use_pytorch:
            tau = M @ q_ddot + C @ q_dot + G
        else:
            M_np = self._from_tensor(M)
            C_np = self._from_tensor(C)
            G_np = self._from_tensor(G)
            q_dot_np = self._from_tensor(q_dot)
            q_ddot_np = self._from_tensor(q_ddot)

            tau = M_np @ q_ddot_np + C_np @ q_dot_np + G_np
            tau = self._to_tensor(tau)

        return tau

    def get_end_effector_pose(self, q: Union[np.ndarray, torch.Tensor]) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get the end-effector position and orientation.

        Args:
            q: Joint positions [n_joints]

        Returns:
            Tuple of (position [3], rotation_matrix [3x3])
        """
        transformations = self.forward_kinematics(q)

        if len(transformations) == 0:
            return np.zeros(3), np.eye(3)

        T_end = transformations[-1]
        position = T_end[:3, 3]
        rotation = T_end[:3, :3]

        return position, rotation

    def print_dynamics_summary(self, q: Union[np.ndarray, torch.Tensor]):
        """Print a summary of the dynamics at a given configuration"""
        q_np = self._from_tensor(q)

        print(f"\n=== Dynamics Summary at q = {q_np} ===")

        # Compute matrices
        M = self._from_tensor(self.compute_mass_matrix(q))
        G = self._from_tensor(self.compute_gravity_vector(q))

        print(f"Mass matrix condition number: {np.linalg.cond(M):.2e}")
        print(f"Mass matrix determinant: {np.linalg.det(M):.2e}")
        print(f"Gravity vector norm: {np.linalg.norm(G):.4f}")

        # End-effector pose
        pos, rot = self.get_end_effector_pose(q)
        print(f"End-effector position: {pos}")
        print(f"End-effector orientation (rotation matrix):\n{rot}")


# Example usage and testing
if __name__ == "__main__":
    # Test with the Panda arm
    urdf_path = "/home/kzl/Projects/RoboGuard/rg_ws/pm/panda_arm.urdf"

    try:
        # Parse URDF
        parser = URDFParser(urdf_path)
        parser.print_summary()

        # Create dynamics model
        robot = RobotDynamics(parser, use_pytorch=False)

        # Test with zero configuration
        q_test = np.zeros(robot.n_joints)
        print("\nTesting dynamics at zero configuration...")
        robot.print_dynamics_summary(q_test)

        # Test forward dynamics
        q_dot = np.zeros(robot.n_joints)
        tau = np.ones(robot.n_joints) * 0.1  # Small torques

        print("\nTesting forward dynamics...")
        q_ddot = robot.forward_dynamics(q_test, q_dot, tau)
        print(f"Joint accelerations: {robot._from_tensor(q_ddot)}")

        # Test inverse dynamics
        print("\nTesting inverse dynamics...")
        tau_computed = robot.inverse_dynamics(q_test, q_dot, q_ddot)
        print(f"Computed torques: {robot._from_tensor(tau_computed)}")
        print(f"Original torques: {tau}")
        print(f"Difference: {robot._from_tensor(tau_computed) - tau}")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
