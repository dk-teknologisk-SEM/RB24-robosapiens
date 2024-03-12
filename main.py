from panda_ros_lib import PandaArm, IterativeParabolicTimeParameterization, rprint
import numpy as np
from copy import deepcopy
from geometry_msgs.msg import Pose, Point, Quaternion

arm = PandaArm()

pose_dict = {
    # pc open tool:
    # position: 
    #     x: 0.056915640501538356
    #     y: 0.6692753067207482
    #     z: 0.05894240640158526
    # orientation: 
    #     x: 0.999772427679532
    #     y: -0.004713052710834839
    #     z: 0.007760531890346116
    #     w: -0.01930425162234236

    "pc_open_tool": Pose(position=Point(x=0.056915640501538356, y=0.6692753067207482, z=0.05594240640158526), orientation=Quaternion(x=0.999772427679532, y=-0.004713052710834839, z=0.007760531890346116, w=-0.01930425162234236)),    

    #screen_frame_remover_tool:
    # position: 
    #     x: 0.11921723907546183
    #     y: 0.6682887607731034
    #     z: 0.05198510034165075
    # orientation: 
    #     x: 0.9997503370695412
    #     y: -0.004974166528565403
    #     z: 0.0032409638284769306
    #     w: -0.021541061954963574

    "screen_frame_remover_tool": Pose(position=Point(x=0.11921723907546183, y=0.6682887607731034, z=0.05198510034165075), orientation=Quaternion(x=0.9997503370695412, y=-0.004974166528565403, z=0.0032409638284769306, w=-0.021541061954963574)), 

    # reattach_screen_frame_tool:
    # position: 
    #     x: 0.16915693824357508
    #     y: 0.6683187553468177
    #     z: 0.062001380081872885
    # orientation: 
    #     x: 0.9996913313731218
    #     y: -0.014610600147982525
    #     z: 0.00577630464276479
    #     w: -0.019245951403475937

    "reattach_screen_frame_tool": Pose(position=Point(x=0.16915693824357508, y=0.6683187553468177, z=0.061001380081872885), orientation=Quaternion(x=0.9996913313731218, y=-0.014610600147982525, z=0.00577630464276479, w=-0.019245951403475937)), 

    #approx_over_pc:
    # position: 
    #     x: 0.22201995933787994
    #     y: 0.12839919454023427
    #     z: 0.0430857855019747
    # orientation: 
    #     x: 0.710601819071197
    #     y: -0.7035943573742
    #     z: 4.96265348249554e-07
    #     w: -0.00018709261380103005
    "approx_over_pc": Pose(position=Point(x=0.22201995933787994, y=0.12839919454023427, z=0.0430857855019747), orientation=Quaternion(x=0.710601819071197, y=-0.7035943573742, z=4.96265348249554e-07, w=-0.00018709261380103005)),

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


    "approx_over_screen": Pose(position=Point(x=0.21498598698384658, y=0.37981437267763063, z=0.04657680654304279), orientation=Quaternion(x=0.7095270195841653, y=-0.7046545877221067, z=-0.003911519093232558, w=0.00424505601643487)),


    # approx_over_screw_tool:
    # joint values: 1.4247089063927794, 0.02294454488027514, -0.33347968736871003, -2.613312880213784, -1.9871708795112801, 1.7851599069568844, -2.7739401184486407
    "approx_over_screw_tool": [1.4311203370395733, 0.01589967568759577, -0.3352690462254318, -2.6189887238850136, -1.9814788819419014, 1.7842299505207273, -2.7759469578915112],


    # above_screen_frame_reattach_startpoint:
    # position: 
    #     x: 0.37860320562584904
    #     y: 0.25827006719757184
    #     z: 0.049199265237291526
    # orientation: 
    #     x: -0.6984942505941352
    #     y: 0.7156153248200037
    #     z: -0.0004737473805072651
    #     w: 0.0005141333660185798


    "above_screen_frame_reattach_startpoint": Pose(position=Point(x=0.37360320562584904, y=0.25827006719757184, z=0.049199265237291526), orientation=Quaternion(x=-0.6984942505941352, y=0.7156153248200037, z=-0.0004737473805072651, w=0.0005141333660185798))    
}

def deg_to_rad(deg):
    return deg * (np.pi / 180)

def rad_to_deg(rad):
    return rad * (180 / np.pi)

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
        arm.rotate(0,0,-np.pi/2)
        arm.align_to_base()
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