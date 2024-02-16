from panda_ros_lib import PandaArm, IterativeParabolicTimeParameterization
import numpy as np

arm = PandaArm()

def move_to_approx_over_pc():
    pose = arm.get_current_pose()
    pose.position.x = 0.5261691584349851
    pose.position.y = -0.1258678959602396
    pose.position.z= 0.2565689323721724

    arm.move_to_joint(pose)

def find_pc_y():
    arm.relative_move(2,0.01)
    arm.relative_move(1,-0.3)
    arm.relative_move(2, -0.02)
    pose = arm.get_current_pose()
    pose.position.y += 0.3
    arm.move_to_contact(pose)

    current_pose = arm.get_current_pose()
    pc_y = current_pose.position.y
    return pc_y

def find_pc_z():
    arm.align_to_base()
    result = arm.move_to_contact()
    if not result[0][2]:
        print("PANIC!!!")
        exit()

    current_pose = arm.get_current_pose()
    pc_z = current_pose.position.z
    return pc_z

def grasp_tool():
    tool_pose = arm.get_current_pose()
    tool_pose.position.x = 0.5719577898768148
    tool_pose.position.y = 0.33215015088761674
    tool_pose.position.z = 0.30
    arm.move_to_joint(tool_pose)
    arm.align_to_base()
    arm.gripper.open()
    tool_pose.position.z = 0.145
    arm.move_to_cartesian(tool_pose)
    arm.align_to_base()
    arm.gripper.grasp(0.02, 40)
    arm.relative_move(2, 0.1)
    

def place_tool():
    arm.align_to_base(z=True)
    tool_pose = arm.get_current_pose()
    tool_pose.position.x = 0.5719577898768148
    tool_pose.position.y = 0.33215015088761674
    tool_pose.position.z = 0.30
    arm.move_to_joint(tool_pose)
    arm.align_to_base()
    tool_pose.position.z = 0.145
    arm.move_to_cartesian(tool_pose)
    arm.gripper.open()
    

def move_to_approx_over_screen():
    # arm.move_to_neutral()
    pose = arm.get_current_pose()
    # neutral as starting position
    pose.position.x = 0.48081756547124316
    pose.position.y = 0.0058284339256326895
    pose.position.z = 0.23626627720711202
    ori = arm.rotate(0,0,-np.pi/2, False)
    pose.orientation = ori
    arm.move_to_joint(pose)


def contact_with_screen_frame():
    arm.align_to_base()
    result = arm.move_to_contact()
    if not result[0][2]:
        print("PANIC!!!")
        exit()  
    current_pose = arm.get_current_pose()
    current_pose.position.y -= 0.2
    arm.move_to_contact(current_pose)
    screen_frame = arm.get_current_pose()  

def remove_screen_frame():
    current_pose = arm.get_current_pose()
    current_pose.position.z += 0.02
    arm.move_to_cartesian(current_pose)
    current_pose = arm.get_current_pose()
    current_pose.position.z += 0.2
    current_pose.orientation = arm.rotate(-70,0,0, False)
    arm.move_to_cartesian(current_pose)



def process_remove_screen_frame():
    grasp_tool()
    move_to_approx_over_screen()
    contact_with_screen_frame()
    remove_screen_frame()
    place_tool()
    

def main():

    arm.clear_error()
    arm.move_to_neutral()
    arm.gripper.close()

    while True:
        move_to_approx_over_pc()
        pc_z = find_pc_z()
        pc_y = find_pc_y()

        arm.relative_move(1, -0.02)
        arm.move_to_neutral()

        grasp_tool()
        move_to_approx_over_screen()
        contact_with_screen_frame()
        remove_screen_frame()
        place_tool()

        arm.move_to_neutral()

        break
    print(f"x: ???, y: {pc_y}, z: {pc_z}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        arm.move_group.stop()