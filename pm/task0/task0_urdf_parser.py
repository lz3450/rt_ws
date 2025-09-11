#!/usr/bin/env python
#
# pm.py
#

"""
URDF Parser for Robotic Arm Dynamics Modeling

This module provides a class to parse URDF files and extract the necessary
parameters for implementing Newton-Euler/Lagrange equations for robotic arm dynamics.



"""

from typing import Dict, List, Optional
from dataclasses import dataclass
import numpy as np
import xml.etree.ElementTree as ET


@dataclass
class LinkInertial:
    """Data class to store inertial properties of a link"""

    mass: float
    center_of_mass: np.ndarray  # [x, y, z]
    inertia_matrix: np.ndarray  # 3x3 inertia tensor

    def __post_init__(self):
        """Ensure proper numpy array shapes"""
        self.center_of_mass = np.array(self.center_of_mass, dtype=np.float64)
        self.inertia_matrix = np.array(self.inertia_matrix, dtype=np.float64)


@dataclass
class JointInfo:
    """Data class to store joint properties"""

    name: str
    joint_type: str
    parent_link: Optional[str]
    child_link: Optional[str]
    origin_xyz: np.ndarray  # Translation [x, y, z]
    origin_rpy: np.ndarray  # Rotation [roll, pitch, yaw]
    axis: np.ndarray  # Joint axis [x, y, z]
    limits: Dict[str, float]  # effort, lower, upper, velocity


class URDFParser:
    """
    A class to parse URDF files and extract parameters needed for
    robotic arm dynamics modeling using Newton-Euler or Lagrange equations.

    This parser extracts:
    - Link inertial properties (mass, center of mass, inertia tensor)
    - Joint information (type, parent/child links, transformations, limits)
    - Kinematic chain structure
    """

    def __init__(self, urdf_file_path: str):
        """
        Initialize the URDF parser with the path to the URDF file.

        Args:
            urdf_file_path (str): Path to the URDF file
        """
        self.urdf_file_path = urdf_file_path
        self.robot_name = ""
        self.links = {}  # Dict[str, LinkInertial]
        self.joints = {}  # Dict[str, JointInfo]
        self.revolute_joints = []  # List of revolute joint names in order
        self.kinematic_chain = []  # Ordered list of links in the kinematic chain

        # Parse the URDF file
        self._parse_urdf()
        self._build_kinematic_chain()

    def _parse_urdf(self):
        """Parse the URDF XML file and extract all relevant information"""
        try:
            tree = ET.parse(self.urdf_file_path)
            root = tree.getroot()

            # Get robot name
            self.robot_name = root.get("name", "unknown_robot")
            print(f"Parsing robot: {self.robot_name}")

            # Parse all links
            self._parse_links(root)

            # Parse all joints
            self._parse_joints(root)

            print(f"Successfully parsed {len(self.links)} links and {len(self.joints)} joints")
            print(f"Found {len(self.revolute_joints)} revolute joints")

        except ET.ParseError as e:
            raise ValueError(f"Error parsing URDF file: {e}")
        except FileNotFoundError:
            raise FileNotFoundError(f"URDF file not found: {self.urdf_file_path}")

    def _parse_links(self, root):
        """Parse all link elements and extract inertial properties"""
        for link_elem in root.findall("link"):
            link_name = link_elem.get("name")

            # Find inertial element
            inertial_elem = link_elem.find("inertial")
            if inertial_elem is not None:
                # Parse mass
                mass_elem = inertial_elem.find("mass")
                mass = float(mass_elem.get("value")) if mass_elem is not None else 0.0

                # Parse center of mass (origin)
                origin_elem = inertial_elem.find("origin")
                com = [0.0, 0.0, 0.0]  # Default center of mass
                if origin_elem is not None:
                    xyz_str = origin_elem.get("xyz", "0 0 0")
                    com = [float(x) for x in xyz_str.split()]

                # Parse inertia matrix
                inertia_elem = inertial_elem.find("inertia")
                inertia_matrix = np.eye(3)  # Default identity matrix
                if inertia_elem is not None:
                    ixx = float(inertia_elem.get("ixx", 0.0))
                    ixy = float(inertia_elem.get("ixy", 0.0))
                    ixz = float(inertia_elem.get("ixz", 0.0))
                    iyy = float(inertia_elem.get("iyy", 0.0))
                    iyz = float(inertia_elem.get("iyz", 0.0))
                    izz = float(inertia_elem.get("izz", 0.0))

                    # Build symmetric inertia matrix
                    inertia_matrix = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])

                # Store link inertial properties
                self.links[link_name] = LinkInertial(
                    mass=mass, center_of_mass=np.array(com), inertia_matrix=inertia_matrix
                )
            else:
                # Link without inertial properties (e.g., end-effector link)
                self.links[link_name] = LinkInertial(
                    mass=0.0, center_of_mass=np.array([0.0, 0.0, 0.0]), inertia_matrix=np.eye(3)
                )

    def _parse_joints(self, root):
        """Parse all joint elements and extract joint information"""
        for joint_elem in root.findall("joint"):
            joint_name = joint_elem.get("name")
            joint_type = joint_elem.get("type")

            # Get parent and child links
            parent_elem = joint_elem.find("parent")
            child_elem = joint_elem.find("child")
            parent_link = parent_elem.get("link") if parent_elem is not None else None
            child_link = child_elem.get("link") if child_elem is not None else None

            # Parse origin (transformation from parent to child)
            origin_elem = joint_elem.find("origin")
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
            if origin_elem is not None:
                xyz_str = origin_elem.get("xyz", "0 0 0")
                rpy_str = origin_elem.get("rpy", "0 0 0")
                xyz = [float(x) for x in xyz_str.split()]
                rpy = [float(x) for x in rpy_str.split()]

            # Parse joint axis (for revolute/prismatic joints)
            axis_elem = joint_elem.find("axis")
            axis = [0.0, 0.0, 1.0]  # Default z-axis
            if axis_elem is not None:
                axis_str = axis_elem.get("xyz", "0 0 1")
                axis = [float(x) for x in axis_str.split()]

            # Parse joint limits
            limits = {}
            limit_elem = joint_elem.find("limit")
            if limit_elem is not None:
                limits["effort"] = float(limit_elem.get("effort", 0.0))
                limits["lower"] = float(limit_elem.get("lower", 0.0))
                limits["upper"] = float(limit_elem.get("upper", 0.0))
                limits["velocity"] = float(limit_elem.get("velocity", 0.0))

            # Store joint information
            joint_info = JointInfo(
                name=joint_name,
                joint_type=joint_type,
                parent_link=parent_link,
                child_link=child_link,
                origin_xyz=np.array(xyz),
                origin_rpy=np.array(rpy),
                axis=np.array(axis),
                limits=limits,
            )

            self.joints[joint_name] = joint_info

            # Keep track of revolute joints in order
            if joint_type == "revolute":
                self.revolute_joints.append(joint_name)

    def _build_kinematic_chain(self):
        """Build the kinematic chain from base to end-effector"""
        # Find the base link (link with no parent joint)
        child_links = set()
        for joint in self.joints.values():
            if joint.child_link:
                child_links.add(joint.child_link)

        # Base link is the one that's not a child of any joint
        base_links = set(self.links.keys()) - child_links
        if not base_links:
            raise ValueError("Could not identify base link")

        base_link = list(base_links)[0]  # Take the first one if multiple

        # Build chain by following joints
        self.kinematic_chain = [base_link]
        current_link = base_link

        while True:
            # Find joint with current_link as parent
            next_joint = None
            for joint in self.joints.values():
                if joint.parent_link == current_link and joint.joint_type != "fixed":
                    next_joint = joint
                    break

            if next_joint is None:
                break

            self.kinematic_chain.append(next_joint.child_link)
            current_link = next_joint.child_link

    def get_dof(self) -> int:
        """Get the degrees of freedom (number of revolute joints)"""
        return len(self.revolute_joints)

    def get_joint_names(self) -> List[str]:
        """Get ordered list of revolute joint names"""
        return self.revolute_joints.copy()

    def get_link_names(self) -> List[str]:
        """Get ordered list of link names in kinematic chain"""
        return self.kinematic_chain.copy()

    def get_link_inertial(self, link_name: str) -> Optional[LinkInertial]:
        """Get inertial properties of a specific link"""
        return self.links.get(link_name)

    def get_joint_info(self, joint_name: str) -> Optional[JointInfo]:
        """Get information about a specific joint"""
        return self.joints.get(joint_name)

    def print_summary(self):
        """Print a summary of the parsed robot structure"""
        print(f"\n=== Robot Summary: {self.robot_name} ===")
        print(f"Degrees of Freedom: {self.get_dof()}")
        print(f"Kinematic Chain: {' -> '.join(self.kinematic_chain)}")

        print("\nRevolute Joints:")
        for i, joint_name in enumerate(self.revolute_joints):
            joint = self.joints[joint_name]
            print(f"  {i + 1}. {joint_name}: {joint.parent_link} -> {joint.child_link}")

        print("\nLink Masses:")
        for link_name in self.kinematic_chain:
            if link_name in self.links:
                mass = self.links[link_name].mass
                print(f"  {link_name}: {mass:.4f} kg")

    def get_transformation_matrix(self, xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
        """
        Create a 4x4 homogeneous transformation matrix from translation and rotation.

        Args:
            xyz: Translation vector [x, y, z]
            rpy: Rotation angles [roll, pitch, yaw] in radians

        Returns:
            4x4 homogeneous transformation matrix
        """
        roll, pitch, yaw = rpy

        # Rotation matrices for each axis
        R_x = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])

        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])

        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

        # Combined rotation matrix (R = R_z * R_y * R_x)
        R = R_z @ R_y @ R_x

        # Build homogeneous transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = xyz

        return T


if __name__ == "__main__":
    # Test the parser with the Panda arm URDF
    urdf_path = "/home/kzl/Projects/RoboGuard/rg_ws/pm/panda_arm.urdf"

    try:
        parser = URDFParser(urdf_path)
        parser.print_summary()

        # Example: Get information about the first joint
        first_joint = parser.get_joint_names()[0]
        joint_info = parser.get_joint_info(first_joint)
        if joint_info:
            print(f"\nExample Joint Info ({first_joint}):")
            print(f"  Type: {joint_info.joint_type}")
            print(f"  Origin XYZ: {joint_info.origin_xyz}")
            print(f"  Origin RPY: {joint_info.origin_rpy}")
            print(f"  Axis: {joint_info.axis}")
            print(f"  Effort limit: {joint_info.limits.get('effort', 'N/A')} Nm")

        # Example: Get inertial properties of the first link
        first_link = parser.get_link_names()[0]
        link_inertial = parser.get_link_inertial(first_link)
        if link_inertial:
            print(f"\nExample Link Inertial ({first_link}):")
            print(f"  Mass: {link_inertial.mass:.4f} kg")
            print(f"  Center of mass: {link_inertial.center_of_mass}")
            print(f"  Inertia matrix:\n{link_inertial.inertia_matrix}")

    except Exception as e:
        print(f"Error: {e}")
