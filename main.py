from panda_ros_lib import PandaArm, IterativeParabolicTimeParameterization, rprint
import numpy as np
from copy import deepcopy
from geometry_msgs.msg import Pose, Point, Quaternion

arm = PandaArm()

pose_dict = {
    # pc open tool:
    # position: 
    #     x: 0.20386157520471365
    #     y: 0.6645801510473288
    #     z: 0.004419399625524723
    # orientation: 
    #     x: 0.9998922253995857
    #     y: 0.0011623080370995616
    #     z: -0.0015716796118986397
    #     w: -0.014550479328488956
    "pc_open_tool": Pose(position=Point(x=0.20386157520471365, y=0.6645801510473288, z=0.005919399625524723), orientation=Quaternion(x=0.9998922253995857, y=0.0011623080370995616, z=-0.0015716796118986397, w=-0.014550479328488956)),
    
    #screen_frame_remover_tool:
    #position: 
    #     x: 0.2623011026342085
    #     y: 0.6659284230551181
    #     z: 0.0005035034893641696
    # orientation: 
    #     x: 0.9998855068696537
    #     y: 0.0018523336344112092
    #     z: -0.0018022485815374601
    #     w: -0.014909524210141535
     "screen_frame_remover_tool": Pose(position=Point(x=0.2623011026342085, y=0.6659284230551181, z=0.0010035034893641696), orientation=Quaternion(x=0.9998855068696537, y=0.0018523336344112092, z=-0.0018022485815374601, w=-0.014909524210141535)),
    #approx_over_pc:
    # position: 
    #     x: 0.2222427201621922
    #     y: 0.1272813651441421
    #     z: 0.09909940894879733
    # orientation: 
    #     x: 0.9998841636265167
    #     y: 0.003217499767674594
    #     z: -0.002832508882793489
    #     w: -0.014604243135989705
    "approx_over_pc": Pose(position=Point(x=0.2222427201621922, y=0.1272813651441421, z=0.12409940894879733), orientation=Quaternion(x=0.9998841636265167, y=0.003217499767674594, z=-0.002832508882793489, w=-0.014604243135989705)),

    #approx_over_screen:
    # position: 
    #     x: 0.21498598698384658
    #     y: 0.37981437267763063
    #     z: 0.04657680654304279
    # orientation: 
    #     x: 0.7095270195841653
    #     y: -0.7046545877221067
    #     z: -0.003911519093232558
    #     w: 0.00424505601643487


    "approx_over_screen": Pose(position=Point(x=0.21498598698384658, y=0.37981437267763063, z=0.04657680654304279), orientation=Quaternion(x=0.7095270195841653, y=-0.7046545877221067, z=-0.003911519093232558, w=0.00424505601643487))
}


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
    arm.rotate(0,0, np.pi/2)
    arm.rotate(0,0, np.pi/2)

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

    arm.move_group.set_end_effector_link("panda_pc_open_tcp")
    arm.rotate(-np.pi/2,0,0)

    arm.move_group.set_end_effector_link("panda_tool_pc_open_tcp")
    arm.relative_move(2, 0.1)
    arm.rotate(np.pi/2,0,0)
    arm.rotate(np.pi/2,0,0)
    arm.relative_move(1, -0.02)
    arm.relative_move(2, -0.11)
    arm.rotate(0,np.pi/2,0)
    arm.rotate(0,np.pi/2,0)

    arm.move_group.set_end_effector_link("panda_pc_open_tcp")
    arm.rotate(-np.pi/2 + 0.2,0,0)
    arm.move_group.set_end_effector_link("panda_tool_pc_open_tcp")
    arm.align_to_base()
    arm.move_to_contact()
    arm.relative_move(2, 0.02)


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
    arm.move_group.set_end_effector_link("panda_hand_tcp")

    arm.move_to_cartesian(tool_pose)
    arm.align_to_base(z=True)
    arm.gripper.open()
    arm.relative_move(2, 0.1)
    

def contact_with_screen_frame():
    arm.align_to_base()
    arm.rotate(0.1,0,0)
    result = arm.move_to_contact()
    if not result[0][2]:
        print("PANIC!!!")
        exit()  
    
    current_pose = arm.get_current_pose()
    current_pose.position.y -= 0.2
    arm.move_to_contact(current_pose, only_in_axis=1)
    screen_frame = arm.get_current_pose()  

def remove_screen_frame():
    # REMOVE FROM EDGE OF SCREEN
    # rprint("START remove_screen_frame")
    # current_pose = arm.get_current_pose()
    # current_pose.position.x += 0.3
    # rprint("FIRST move_to_contact")
    # arm.move_to_contact(current_pose, only_in_axis=0)
    # rprint("SECOND move_to_contact")
    # arm.move_to_contact(current_pose, only_in_axis=0)
    # arm.clear_error()

    # REMOVE FROM MIDDLE OF SCREEN
    current_pose = arm.get_current_pose()
    current_pose.position.z += 0.02
    arm.move_to_cartesian(current_pose)
    
    arm.relative_move(2, 0.02)
    current_pose = arm.get_current_pose()
    current_pose.position.z += 0.2
    current_pose.orientation = arm.rotate(-0.7,0,0, False)
    arm.move_to_cartesian(current_pose)


def main():

    arm.clear_error()
    arm.move_to_neutral()
    arm.gripper.close()

    while True:
        arm.align_to_base(z=True)
        grasp_tool(pose_dict["pc_open_tool"])
        arm.move_group.set_end_effector_link("panda_tool_pc_open_tcp")
        arm.move_to_cartesian(pose_dict["approx_over_pc"])
        
        pc_closed_z = find_pc_z()
        # pc_closed_x = find_pc_x() NOT WORKING RIGHT NOW, TOO CLOSE TO JOINT LIMITS
        arm.relative_move(2, 0.02)
        arm.rotate(0,0,np.pi/2)
        pc_closed_y = find_pc_y()
        open_pc(pc_closed_y, pc_closed_z)

        place_tool(tool_pose=pose_dict["pc_open_tool"])

        grasp_tool(pose_dict["screen_frame_remover_tool"])
        arm.move_group.set_end_effector_link("panda_tool_screen_frame_remover_tcp")
        arm.move_to_joint(pose_dict["approx_over_screen"])
        contact_with_screen_frame()
        remove_screen_frame()
        place_tool(pose_dict["screen_frame_remover_tool"])
        
        arm.move_to_neutral()

        break


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        arm.move_group.stop()