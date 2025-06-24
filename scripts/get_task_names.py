PLATFORMS = ["sim", "phy"]
ROBOT_TYPES = ["wheeled", "arm"]
ROBOT_TASKS = {
    "wheeled": ["navigating", "docking", "following"],
    "arm": ["picking", "placing", "drawing"],
}
ATTACK_VECTORS = ["config", "rogue_node", "algorithm", "actuator"]

for platform in PLATFORMS:
    for robot_type in ROBOT_TYPES:
        for task in ROBOT_TASKS[robot_type]:
            for attack_vector in ATTACK_VECTORS:
                print(f"{platform}-{robot_type}-{task}-{attack_vector}")
