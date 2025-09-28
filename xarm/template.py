#!/usr/bin/env python
#
# template.py
#


import sys
from configparser import ConfigParser

from xarm.wrapper import XArmAPI


def run(arm: XArmAPI) -> None:
    print("=" * 50)
    print("default_is_radian:", arm.default_is_radian)
    print("version:", arm.version)
    print("state:", arm.state)
    print("mode:", arm.mode)
    print("cmd_num:", arm.cmd_num)
    print("error_code:", arm.error_code)
    print("warn_code:", arm.warn_code)
    print("collision_sensitivity:", arm.collision_sensitivity)
    print("teach_sensitivity:", arm.teach_sensitivity)
    print("world_offset:", arm.world_offset)
    print("gravity_direction:", arm.gravity_direction)

    print("============TCP============")
    print("* position:", arm.position)
    print("* tcp_jerk:", arm.tcp_jerk)
    print("* tcp_load:", arm.tcp_load)
    print("* tcp_offset:", arm.tcp_offset)
    print("* tcp_speed_limit:", arm.tcp_speed_limit)
    print("* tcp_acc_limit:", arm.tcp_acc_limit)

    print("===========JOINT===========")
    print("* angles:", arm.angles)
    print("* joint_jerk:", arm.joint_jerk)
    print("* joint_speed_limit:", arm.joint_speed_limit)
    print("* joint_acc_limit:", arm.joint_acc_limit)
    print("* joints_torque:", arm.joints_torque)


def error_warn_handler(item: dict) -> None:
    print(f"ErrorCode: {item['error_code']}, WarnCode: {item['warn_code']}")
    # TODO：Do different processing according to the error code


def main() -> None:
    """
    Description:
        1. Instantiate XArmAPI and specify do_not_open to be true
        2. Registration error callback function
        3. Connect
        4. Enable motion
        5. Setting mode
        6. Setting state
    """
    parser = ConfigParser()
    try:
        parser.read("lite6.conf")
        ip = parser.get("xArm", "ip")
    except Exception as e:
        print(f"Error reading configuration: {e}")
        ip = input("Please input the xArm ip address:")
        if not ip:
            print("input error, exit")
            sys.exit(1)

    arm = XArmAPI(ip, do_not_open=True)
    arm.register_error_warn_changed_callback(error_warn_handler)
    arm.connect()

    arm.motion_enable(enable=True)
    arm.set_mode(mode=0)
    arm.set_state(state=0)

    run(arm)

    arm.disconnect()


if __name__ == "__main__":
    main()
