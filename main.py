from panda_ros_lib import PandaArm, IterativeParabolicTimeParameterization
import numpy as np
from copy import deepcopy

arm = PandaArm()

def move_to_approx_over_pc():
    pose = arm.get_current_pose()
    pose.position.x = 0.5261691584349851
    pose.position.y = -0.2258678959602396
    pose.position.z= 0.1405689323721724

    arm.move_to_joint(pose)

def find_pc_z():
    arm.align_to_base()
    result = arm.move_to_contact()
    if not result[0][2]:
        print("PANIC!!!")
        exit()

    current_pose = arm.get_current_pose()
    pc_z = current_pose.position.z
    return pc_z

def find_pc_x():
    arm.relative_move(2,0.01)
    arm.relative_move(0,-0.25)
    arm.relative_move(2, -0.04)
    pose = arm.get_current_pose()
    pose.position.x += 0.3
    arm.move_to_contact(pose)

    current_pose = arm.get_current_pose()
    pc_x_min = current_pose.position.x

    arm.relative_move(2,0.04)
    arm.relative_move(0,0.45)
    arm.rotate(0,0,np.pi)
    arm.relative_move(2, -0.04)
    pose = arm.get_current_pose()
    pose.position.x -= 0.3
    arm.move_to_contact(pose)

    current_pose = arm.get_current_pose()
    pc_x_max = current_pose.position.x

    pc_x_center = (pc_x_min + pc_x_max) / 2

    arm.relative_move(2,0.04)
    current_pose = arm.get_current_pose()
    current_pose.position.x = pc_x_center
    arm.move_to_cartesian(current_pose)
    arm.rotate(0,0,np.pi/2)

    return {"center":pc_x_center, "min":pc_x_min, "max":pc_x_max}

def find_pc_y():
    arm.relative_move(1,-0.15)
    arm.relative_move(2, -0.04)
    pose = arm.get_current_pose()
    pose.position.y += 0.3
    arm.move_to_contact(pose)

    current_pose = arm.get_current_pose()
    pc_y = current_pose.position.y
    return pc_y

def open_pc(pc_closed_y, pc_closed_z):
    arm.relative_move(1,-0.03)
    arm.rotate(0,0,-np.pi)

    pc_lid_thickness = 0.007
    current_pose = arm.get_current_pose()
    current_pose.position.z = pc_closed_z - pc_lid_thickness
    arm.move_to_cartesian(current_pose)

    current_pose = arm.get_current_pose()
    current_pose.position.y = pc_closed_y 
    arm.move_to_cartesian(current_pose)
    
    current_pose = arm.get_current_pose()
    current_pose.position.y = 0.05
    arm.move_to_contact(current_pose)

    current_pose = arm.get_current_pose()
    current_pose.position.z += 0.21
    current_pose.position.y += 0.06
    ori = arm.rotate(-0.2,0,0, False)
    current_pose.orientation = ori
    arm.move_to_cartesian(current_pose)


def get_pc_open_tool():
    pose = arm.get_current_pose()
    pose.position.x = 0.5020610183734253
    pose.position.y = 0.33300988946444704
    pose.position.z= 0.0338577209878773
    return pose

def get_screen_frame_remover_tool():
    pose = arm.get_current_pose()
    pose.position.x = 0.5719577898768148
    pose.position.y = 0.33215015088761674
    pose.position.z = 0.0308577209878773
    return pose

def grasp_tool(tool_pose):
    tool_pose_above = deepcopy(tool_pose)
    tool_pose_above.position.z += 0.1
    arm.move_to_joint(tool_pose_above)
    arm.align_to_base()
    arm.gripper.open()

    arm.move_to_cartesian(tool_pose)
    arm.align_to_base()
    arm.gripper.grasp(0.02, 40)
    arm.relative_move(2, 0.1)
    

def place_tool(tool_pose):
    arm.align_to_base(z=True)
    tool_pose_above = deepcopy(tool_pose)
    tool_pose_above.position.z += 0.1
    arm.move_to_joint(tool_pose_above)
    

    arm.move_to_cartesian(tool_pose)
    arm.align_to_base(z=True)
    arm.gripper.open()
    arm.relative_move(2, 0.1)
    

def move_to_approx_over_screen():
    # arm.move_to_neutral()
    pose = arm.get_current_pose()
    # neutral as starting position
    pose.position.x = 0.48081756547124316
    pose.position.y = 0.0058284339256326895
    pose.position.z = 0.12026627720711202
    ori = arm.rotate(0,0,-np.pi/2, False)
    pose.orientation = ori
    arm.move_to_joint(pose)


def contact_with_screen_frame():
    arm.align_to_base()
    result = arm.move_to_contact()
    if not result[0][2]:
        print("PANIC!!!")
        exit()  
    
    arm.align_to_base()
    arm.move_to_contact()
    current_pose = arm.get_current_pose()
    current_pose.position.y -= 0.2
    ori = arm.rotate(0.1,0,0, False)
    current_pose.orientation = ori
    arm.move_to_contact(current_pose)
    screen_frame = arm.get_current_pose()  

def remove_screen_frame():
    current_pose = arm.get_current_pose()
    current_pose.position.z += 0.02
    arm.move_to_cartesian(current_pose)
    current_pose = arm.get_current_pose()
    current_pose.position.z += 0.2
    current_pose.orientation = arm.rotate(-0.7,0,0, False)
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
        arm.align_to_base(z=True)
        grasp_tool(get_pc_open_tool())
        move_to_approx_over_pc()
        pc_closed_z = find_pc_z()
        # pc_closed_x = find_pc_x() NOT WORKING RIGHT NOW, TOO CLOSE TO JOINT LIMITS
        arm.relative_move(2, 0.02)
        arm.rotate(0,0,np.pi/2)
        pc_closed_y = find_pc_y()
        open_pc(pc_closed_y, pc_closed_z)

        # arm.relative_move(2, 0.2)
        arm.relative_move(1, -0.05)
        arm.move_to_neutral()
        place_tool(get_pc_open_tool())

        grasp_tool(get_screen_frame_remover_tool())
        move_to_approx_over_screen()
        contact_with_screen_frame()
        remove_screen_frame()
        arm.align_to_base(z=True)
        arm.rotate(0,0,np.pi)
        place_tool(get_screen_frame_remover_tool())

        arm.move_to_neutral()

        break


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        arm.move_group.stop()