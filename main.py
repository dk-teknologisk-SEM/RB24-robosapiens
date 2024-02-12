from panda_ros_lib import PandaArm



arm = PandaArm()

def move_to_approx_over_pc():
    pose = arm.get_current_pose()
    pose.position.x = 0.5261691584349851
    pose.position.y = -0.1258678959602396
    pose.position.z= 0.2565689323721724

    arm.move_to_cartesian(pose)


def main():

    arm.clear_error()
    arm.move_to_neutral()

    while True:
        move_to_approx_over_pc()

        result = arm.move_to_contact()
        if not result[0][2]:
            print("PANIC!!!")
            exit()

        current_pose = arm.get_current_pose()
        pc_z = current_pose.position.z

        arm.relative_move(1,-0.3)
        arm.relative_move(2, -0.01)
        pose = arm.get_current_pose()
        pose.position.y += 0.3
        arm.move_to_contact(pose)

        current_pose = arm.get_current_pose()
        pc_y = current_pose.position.y



        break
    print(f"x: ???, y: {pc_y}, z: {pc_z}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        arm.move_group.stop()