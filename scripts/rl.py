#!/usr/bin/env python3

import subprocess
import re

EXCLUDE_RE = {
    "nodes": [
        r"/_\S+",
        r"/\S+_private_\S+",
        r"/analyzers",
        r"/launch_ros_\S+",
        # r"/transform_listener_impl_\S+",
    ],
    "topics": [
        r"/bond",
        r"/clock",
        r"/diagnostics.*",
        r"/parameter_events",
        r"/rosout",
        # r"/tf",
    ],
    "services": [
        r"/\S+/describe_parameters",
        r"/\S+/get_parameter_types",
        r"/\S+/get_parameters",
        r"/\S+/list_parameters",
        r"/\S+/set_parameters_atomically",
        r"/\S+/set_parameters",
    ],
    "actions": [],
}


def run_command(command: list[str]) -> str:
    """Run a command and return its output."""
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"Error running command: {' '.join(command)}\nstderr:\n{result.stderr}")
    return result.stdout


def list_nodes() -> list[str]:
    """Run 'ros2 node list' to get all online nodes."""
    command = ["ros2", "node", "list", "-a"]
    output = run_command(command)
    if not output:
        raise RuntimeError("No ROS2 nodes found.")
    nodes = output.splitlines()
    return [node for node in nodes if not any(re.fullmatch(pattern, node) for pattern in EXCLUDE_RE["nodes"])]


def get_node_info(node: str) -> str:
    """Run 'ros2 node info <node>' to get detailed info of the node."""
    command = ["ros2", "node", "info", node]
    try:
        output = run_command(command)
    except RuntimeError as e:
        print(f"Warning: {e}")
        output = ""
    return output


def parse_node_info(node_info_output: str) -> dict[str, list]:
    # {role: [medium_name, ...]}
    # role: Subscribers, Publishers, Service Servers, Service Clients, Action Servers, Action Clients
    node_info: dict[str, list] = {}

    role = None
    patterns: list[re.Pattern] = []
    topic_exclude_patterns: list[re.Pattern] = [re.compile(pattern) for pattern in EXCLUDE_RE["topics"]]
    service_exclude_patterns: list[re.Pattern] = [re.compile(pattern) for pattern in EXCLUDE_RE["services"]]
    action_exclude_patterns: list[re.Pattern] = [re.compile(pattern) for pattern in EXCLUDE_RE["actions"]]
    for line in node_info_output.strip().splitlines():
        line = line.strip()

        if not line:
            continue

        if line.endswith(":"):
            role = line[:-1]
            node_info[role] = []
            match role:
                case "Subscribers" | "Publishers":
                    patterns = topic_exclude_patterns
                case "Service Servers" | "Service Clients":
                    patterns = service_exclude_patterns
                case "Action Servers" | "Action Clients":
                    patterns = action_exclude_patterns
                case _:
                    raise ValueError(f"Unknown section: {role}")
        elif role:
            medium = line.split(": ")[0]
            if not any(pattern.fullmatch(medium) for pattern in patterns):
                node_info[role].append(medium)
        else:
            # first line is the node name
            print(f"Parsing node '{line}'...")

    return node_info


def generate_dot_file(node_roles: dict[str, dict[str, list]], output_file: str):
    """
    Generate a .dot file to visualize node relationships.
    `medium`: topic, service, action
    """

    media_nodes: dict[str, dict[str, list]] = {}

    for node, roles in node_roles.items():
        for role, media in roles.items():
            for medium in media:
                if medium not in media_nodes:
                    media_nodes[medium] = {}
                if role not in media_nodes[medium]:
                    media_nodes[medium][role] = []
                if node not in media_nodes[medium][role]:
                    media_nodes[medium][role].append(node)

    with open(output_file, "w") as f:
        f.write("digraph {\n")
        f.write("    rankdir = LR;\n")

        for node in node_roles.keys():
            f.write(f'    "{node}" [color = black;penwidth = 2;];\n')

        for medium, roles in media_nodes.items():
            for role, nodes in roles.items():
                reverse = False
                match role:
                    case "Publishers":
                        color = "blue"
                    case "Subscribers":
                        color = "blue"
                        reverse = True
                    case "Service Servers":
                        color = "green"
                        reverse = True
                    case "Service Clients":
                        color = "green"
                    case "Action Servers":
                        color = "red"
                        reverse = True
                    case "Action Clients":
                        color = "red"
                    case _:
                        raise ValueError(f"Unknown relation: {role}")

                f.write(f'    "{medium}" [shape = rectangle;color = {color};];\n')
                for n in nodes:
                    if reverse:
                        f.write(f'    "{medium}" -> "{n}" [color = {color};];\n')
                    else:
                        f.write(f'    "{n}" -> "{medium}" [color = {color};];\n')
        f.write("}\n")


def main():
    nodes = list_nodes()
    print(f"# Nodes: {len(nodes)}")

    # {node_name: {role: [medium_name, ...], ...}, ...}
    node_roles = {}

    for node in nodes:
        info = get_node_info(node)
        parsed_info = parse_node_info(info)
        if any(parsed_info.values()):
            node_roles[node] = parsed_info

    generate_dot_file(node_roles, "ros2_graph.dot")


if __name__ == "__main__":
    main()
