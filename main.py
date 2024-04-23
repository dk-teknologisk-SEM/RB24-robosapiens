from panda_ros_lib import PandaArm, IterativeParabolicTimeParameterization, rprint
import numpy as np
from copy import deepcopy
from geometry_msgs.msg import Pose, Point, Quaternion
from time import sleep
from math import dist
import rospy

arm = PandaArm()

pose_dict = {
    # pc open tool:
    # position: 
    #     x: 0.05733242432911795
    #     y: 0.6689132445003902
    #     z: 0.05685740570286763
    # orientation: 
    #     x: 0.999749275808511
    #     y: -0.007967242863267691
    #     z: 0.007470049325425549
    #     w: -0.019547555463311283


    "pc_open_tool": Pose(position=Point(x=0.05733242432911795, y=0.6689132445003902, z=0.05685740570286763 + 0.004), orientation=Quaternion(x=0.999749275808511, y=-0.007967242863267691, z=0.007470049325425549, w=-0.019547555463311283)),    

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
    #     x: 0.2007785586355047
    #     y: 0.3786916359605762
    #     z: 0.038138931532571134
    # orientation: 
    #     x: 0.7127143494800567
    #     y: -0.7014080661131809
    #     z: 0.004175902436892146
    #     w: -0.006895119681679882

    "approx_over_screen": Pose(position=Point(x=0.2007785586355047, y=0.3786916359605762, z=0.038138931532571134), orientation=Quaternion(x=0.7127143494800567, y=-0.7014080661131809, z=0.004175902436892146, w=-0.006895119681679882)),   

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

    "above_screen_frame_reattach_startpoint": Pose(position=Point(x=0.37360320562584904, y=0.25827006719757184, z=0.049199265237291526), orientation=Quaternion(x=-0.6984942505941352, y=0.7156153248200037, z=-0.0004737473805072651, w=0.0005141333660185798)),

    "approx_above_screen_with_screw_tool": [1.8009205215855648, -0.9884424066794545, -1.8641107601701168, -2.083275166394418, 0.7750173774303746, 2.066743081026607, -1.4132072120328094],

    "just_above_screw1": [2.0513957632281565, -1.3186599237709715, -2.121399429253227, -1.5877565826449473, 0.9235448146441917, 2.356821927640176, -1.571307454506556],

    "just_above_screw2": [1.7321933018323126, -1.0553358802293475, -2.2611543609124105, -1.9984467784145419, 0.3457644094769723, 1.6139093490574095, -1.580690213465264],

    "just_above_screw3": [2.1283724424728185, -1.0860230510110789, -2.1820111963031374, -2.0463589438466836, 0.4162229795191023, 1.7757747641285082, -1.5331130959053296],
    # if not new fixture with corners "just_above_screw3": [1.9068539003740277, -1.0985445564504255, -2.0992789021548406, -2.314217004943312, 0.5971005605599946, 2.1517024513218135, -1.651512649972728],

    "just_above_screw4": [2.4948734209014707, -1.308439919890019, -2.2472216971774386, -1.332582924396371, 0.8167961779170564, 2.071720024360551, -1.4336195604183124],
    # if not new fixture with corners "just_above_screw4": [2.4193914513609065, -1.320201842140733, -2.1651112390362925, -1.4635632465885928, 0.8576647046496799, 2.2705184519496253, -1.499695961078008],

    #screw hole1
    # position: 
    #     x: 0.1600621267172057
    #     y: 0.6878034208467507
    #     z: 0.03463501270756624
    # orientation: 
    #     x: 0.6887049373286737
    #     y: 0.7250293010090045
    #     z: -0.003995663497452338
    #     w: 0.0014341027569972175

    "screw_hole1": Pose(position=Point(x=0.1600621267172057, y=0.6878034208467507, z=0.03463501270756624), orientation=Quaternion(x=0.6887049373286737, y=0.7250293010090045, z=-0.003995663497452338, w=0.0014341027569972175)),   

    #screw hole2
    # position: 
    #     x: 0.16030717803467887
    #     y: 0.6729645361788448
    #     z: 0.034563602065208525
    # orientation: 
    #     x: 0.6886717192291175
    #     y: 0.7250591639811609
    #     z: -0.004285066530224
    #     w: 0.0014526065481268717

    "screw_hole2": Pose(position=Point(x=0.16030717803467887, y=0.6729645361788448, z=0.034563602065208525), orientation=Quaternion(x=0.6886717192291175, y=0.7250591639811609, z=-0.004285066530224, w=0.0014526065481268717)),    

    #screw hole3
    # position: 
    #     x: 0.16062610829692997
    #     y: 0.6575569409998826
    #     z: 0.034488979395802775
    # orientation: 
    #     x: 0.6872320131867741
    #     y: 0.7263947624655684
    #     z: -0.007402852501839142
    #     w: 0.002829644620795101

    "screw_hole3": Pose(position=Point(x=0.16062610829692997, y=0.6575569409998826, z=0.034488979395802775), orientation=Quaternion(x=0.6872320131867741, y=0.7263947624655684, z=-0.007402852501839142, w=0.002829644620795101)),  

    # screw hole4
    # position: 
    #     x: 0.1607341723794684
    #     y: 0.6429432610064418
    #     z: 0.03466837774577459
    # orientation: 
    #     x: 0.6882722406461264
    #     y: 0.725428553366464
    #     z: -0.005629337373660535
    #     w: 0.00174564524859849
                                  
    "screw_hole4": Pose(position=Point(x=0.1607341723794684, y=0.6429432610064418, z=0.03466837774577459), orientation=Quaternion(x=0.6882722406461264, y=0.725428553366464, z=-0.005629337373660535, w=0.00174564524859849)),  

    # pc open pose
    # position: 
    #     x: 0.22488338974878735
    #     y: 0.5337942703884697
    #     z: 0.012429523185995345
    # orientation: 
    #     x: 0.7115514192389913
    #     y: -0.7026241814753624
    #     z: 0.0017259332275041644
    #     w: -0.0032952298172670823

    "pc_open_pose": Pose(position=Point(x=0.22488338974878735, y=0.5337942703884697, z=0.012429523185995345), orientation=Quaternion(x=0.7115514192389913, y=-0.7026241814753624, z=0.0017259332275041644, w=-0.0032952298172670823)),  

    # screen remover tool
    # position: 
    #     x: -0.09752514464401774
    #     y: 0.6699170123947952
    #     z: 0.052242802087636785
    # orientation: 
    #     x: 0.9998842004260421
    #     y: 0.011519073565381807
    #     z: -0.0011068183837063593
    #     w: -0.00988289611579314

    "screen_remover_tool": Pose(position=Point(x=-0.09752514464401774, y=0.6699170123947952, z=0.052242802087636785), orientation=Quaternion(x=0.9998842004260421, y=0.011519073565381807, z=-0.0011068183837063593, w=-0.00988289611579314)),  

    "screen_remover_init_pose" : [1.2586937529445135, 1.141014569198876, -1.5285425722009187, -1.963085132983693, 1.1754174835417006, 1.8310459912037844, 1.7666723326858293],

    "screen_frame_holder_pose": [-0.3372660331767902, 0.908194028473517, 0.7066221270561218, -1.2979891606535983, 1.1430772891574437, 1.9559269683123433, -1.2271082312517112],

    # replace screen frame pose
    # position: 
    #     x: 0.20760234949738338
    #     y: 0.2666872894619938
    #     z: 0.021453934618378794
    # orientation: 
    #     x: 0.00890364091094405
    #     y: 0.9998229758075018
    #     z: -0.016138900135312096
    #     w: 0.0037786410762532174

    "replace_screen_frame_pose": Pose(position=Point(x=0.20760234949738338, y=0.2666872894619938, z=0.021453934618378794), orientation=Quaternion(x=0.00890364091094405, y=0.9998229758075018, z=-0.016138900135312096, w=0.0037786410762532174)),  

    # setpoint replace screen frame
    # position: 
    #     x: 0.20539675969660026
    #     y: 0.16133011533858677
    #     z: 0.10627113002249214
    # orientation: 
    #     x: 0.010050573066068973
    #     y: 0.9998044633837311
    #     z: -0.016748926566094887
    #     w: 0.0030813045771607393

    "setpoint_replace_screen_frame": Pose(position=Point(x=0.20539675969660026, y=0.16133011533858677, z=0.10627113002249214), orientation=Quaternion(x=0.010050573066068973, y=0.9998044633837311, z=-0.016748926566094887, w=0.0030813045771607393)), 
    
    # pickup screen frame from holder
    # position: 
    #     x: 0.3505188466055554
    #     y: 0.7079726656182903
    #     z: 0.2872879247477031
    # orientation: 
    #     x: -0.6955171374323214
    #     y: 0.007073613147483974
    #     z: 0.014305651282145912
    #     w: 0.7183322517306203

    "pickup_screen_frame_from_holder": Pose(position=Point(x=0.3505188466055554, y=0.7079726656182903, z=0.2872879247477031), orientation=Quaternion(x=-0.6955171374323214, y=0.007073613147483974, z=0.014305651282145912, w=0.7183322517306203)), 

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

    move_list = [arm.rotate(0,0, np.pi/2, False), arm.rotate(0,0, np.pi, False)]
    arm.move_to_cartesian(move_list)

    pc_lid_thickness = 0.007
    current_pose = arm.get_current_pose()
    current_pose.position.z = pc_closed_z - pc_lid_thickness - 0.004
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

    current_pose = arm.get_current_pose()
    rotation_list = [arm.rotate(np.pi/2,0,0, False),arm.rotate(np.pi,np.pi/2,0, False), arm.rotate(np.pi,np.pi,0, False)]
    arm.move_to_cartesian(rotation_list)

    arm.relative_move(1, -0.02)
    arm.relative_move(2, -0.11)

    arm.move_group.set_end_effector_link("panda_pc_open_tcp")
    arm.rotate(-np.pi/2 + deg_to_rad(15),0,0)
    arm.move_group.set_end_effector_link("panda_tool_pc_open_tcp")
    arm.align_to_base()
    arm.move_to_contact()
    arm.relative_move(2, 0.02)
    
    return arm.get_current_pose()

def close_pc(pc_open_pose):
    target_pose = deepcopy(pc_open_pose)
    target_pose.position.y += 0.08
    target_pose.position.z -= 0.025
    arm.move_to_cartesian(target_pose)

    current_pose = arm.get_current_pose()
    current_pose.position.y -= 0.05
    arm.move_to_contact(current_pose)

    arm.move_group.set_end_effector_link("panda_pc_open_tcp")
    arm.rotate(np.pi/2,0,0)

    arm.move_group.set_end_effector_link("panda_tool_pc_open_tcp")
    arm.relative_move(2, 0.1)
    current_pose = arm.get_current_pose()
    rotation_lst = [arm.rotate(-np.pi/2,0,0, False),arm.rotate(-np.pi,np.pi/2,0, False), arm.rotate(-np.pi,np.pi,0, False)]
    arm.move_to_cartesian(rotation_lst)
    arm.relative_move(1, 0.02)
    arm.relative_move(2, -0.11)

    arm.move_group.set_end_effector_link("panda_pc_open_tcp")
    arm.rotate(np.pi/2 - deg_to_rad(10),0,0)
    arm.move_group.set_end_effector_link("panda_tool_pc_open_tcp")
    arm.align_to_base()
    arm.relative_move(2, 0.02)

def wait_for_move_complete(goal_pose, tolerance=0.005):
    current_pose = arm.get_current_pose()
    d = dist([current_pose.position.x, current_pose.position.y, current_pose.position.z], [goal_pose.position.x, goal_pose.position.y, goal_pose.position.z])
    while  d > tolerance:
        sleep(1)
        current_pose = arm.get_current_pose()
        d = dist([current_pose.position.x, current_pose.position.y, current_pose.position.z], [goal_pose.position.x, goal_pose.position.y, goal_pose.position.z])

def reattach_screen_frame(force_z):
    rprint("start reattach_screen_frame")
    arm.start_cartestion_impedance_controller([1000,1000,1000,10,10,10], [1.0,1.0,1.0,1.0,1.0,1.0])

    arm.move_to_contact(speed=0.005)
    
    rprint("set_cartestion_impedance_wrench")
    arm.set_cartestion_impedance_wrench([0,0,force_z], [0,0,0])  
    sleep(1)

    # rprint("first move")
    # goal_pose = arm.relative_move(1, 0.25)
    # wait_for_move_complete(goal_pose=goal_pose, tolerance=0.01)

    # rprint("second move")
    # goal_pose = arm.relative_move(0, -0.37)
    # wait_for_move_complete(goal_pose=goal_pose, tolerance=0.01)

    # rprint("third move")
    # goal_pose = arm.relative_move(1, -0.24)
    # wait_for_move_complete(goal_pose=goal_pose, tolerance=0.01)

    # rprint("fourth move")
    # goal_pose = arm.relative_move(0, 0.37)
    # wait_for_move_complete(goal_pose=goal_pose, tolerance=0.01)

def reattach_screen_frame(force_z):
    rprint("start reattach_screen_frame")
    arm.start_cartestion_impedance_controller([1000,1000,1000,10,10,10], [1.0,1.0,1.0,1.0,1.0,1.0])
    arm.move_to_contact(speed=0.005)
    rprint("set_cartestion_impedance_wrench")
    arm.set_cartestion_impedance_wrench([0,0,force_z], [0,0,0])  
    sleep(1)
    rprint("first move")
    goal_pose = arm.relative_move(0, -0.355)
    wait_for_move_complete(goal_pose=goal_pose, tolerance=0.01)
    rprint("move done")
    rprint("second move")
    goal_pose = arm.relative_move(1, 0.25)
    wait_for_move_complete(goal_pose=goal_pose, tolerance=0.024)
    rprint("third move")
    goal_pose = arm.get_current_pose()
    goal_pose.position.x += 0.36
    goal_pose.position.y -= 0.007
    arm.move_to_cartesian(goal_pose)
    wait_for_move_complete(goal_pose=goal_pose, tolerance=0.015)
    rprint("fourth move")
    goal_pose = arm.relative_move(1, -0.22)
    wait_for_move_complete(goal_pose=goal_pose, tolerance=0.01)
    rprint("stop_cartestion_impedance_controller")
    arm.set_cartestion_impedance_wrench([0,0,0], [0,0,0])
    arm.stop_cartestion_impedance_controller()
    arm.relative_move(2, 0.02)

def remove_screen():
    current_pose = arm.get_current_pose()
    current_pose.position.z += 0.35
    current_pose.position.y -= 0.3
    current_pose.position.x += 0.2
    arm.move_to_cartesian(current_pose)
    arm.move_to_joint(pose_dict["screen_remover_init_pose"])
    arm.move_to_cartesian(pose_dict["approx_over_screen"])
    arm.rotate(0.02,0,0)
    old_threshold = arm.lower_force
    set_force_contact_threshold([20,20,20,23,23,23])
    arm.move_to_contact()
    arm.relative_move(2, 0.05)
    current_joint_pose = list(arm.state.q)
    current_joint_pose[6] -= np.pi
    arm.move_to_joint(current_joint_pose)
    current_pose = arm.get_current_pose()
    current_pose.position.x -= 0.41
    current_pose.position.y -= 0.33
    arm.move_to_cartesian(current_pose)
    
    set_force_contact_threshold(old_threshold)
    arm.relative_move(2, -0.05)
    arm.move_to_contact()
    current_pose = arm.get_current_pose()
    current_pose.position.y -= 0.03
    arm.move_to_contact(current_pose, only_in_axis=0)
    arm.clear_error()
    current_pose = arm.get_current_pose()
    current_pose.position.x += 0.02
    current_pose.position.y -= 0.0025
    arm.move_to_contact(current_pose, only_in_axis=1)
    dropoff_screen_pose = arm.get_current_pose()
    set_force_contact_threshold(force_contact_threshold=old_threshold)
    arm.clear_error()
    
    arm.move_group.set_end_effector_link("panda_tool_screen_remover_tcp_off")
    arm.rotate(deg_to_rad(-10),0,0)
    arm.relative_move(2, 0.05)
    arm.move_group.set_end_effector_link("panda_tool_screen_remover_tcp")

    return dropoff_screen_pose

def pickup_screen_from_holder(dropoff_screen_pose):
    dropoff_screen_pose_above = deepcopy(dropoff_screen_pose)
    dropoff_screen_pose_above.position.z += 0.05
    arm.move_to_cartesian(dropoff_screen_pose_above)

    old_threshold = arm.lower_force
    set_force_contact_threshold([35,35,35,38,38,38])
    arm.rotate(-0.02, 0, 0)
    arm.rotate(0,0.02, 0)
    arm.move_to_contact()

    current_pose = arm.get_current_pose()
    current_pose.position.x -= 0.01
    current_pose.position.y += 0.04
    arm.move_to_cartesian(current_pose)
    arm.relative_move(2, 0.1)
    set_force_contact_threshold(old_threshold)

def replace_screen():
    current_joint_pose = list(arm.state.q)
    current_joint_pose[6] += np.pi
    arm.move_to_joint(current_joint_pose)
    arm.move_to_cartesian(pose_dict["approx_over_screen"])
    arm.relative_move(0,-0.0015)
    arm.move_to_contact()
    rprint(arm.robot_mode)
    arm.clear_error()

    old_threshold = arm.lower_force
    set_force_contact_threshold([15,15,15,18,18,18])
    arm.move_group.set_end_effector_link("panda_tool_screen_remover_tcp_off")
    arm.rotate(deg_to_rad(10),0,0)
    arm.relative_move(2, 0.1)
    arm.move_group.set_end_effector_link("panda_tool_screen_remover_tcp")
    set_force_contact_threshold(old_threshold)

def grasp_tool(tool_pose):
    tool_pose_above = deepcopy(tool_pose)
    tool_pose_above.position.z += 0.1
    arm.move_to_joint(tool_pose_above)
    arm.gripper.open()

    arm.move_to_cartesian(tool_pose)
    arm.gripper.grasp(0.02, 40)
    arm.relative_move(2, 0.1)
    
def place_tool(tool_pose):
    tool_pose_above = deepcopy(tool_pose)
    tool_pose_above.position.z += 0.1
    arm.move_to_joint(tool_pose_above)
    arm.move_group.set_end_effector_link("panda_hand_tcp")

    arm.move_to_cartesian(tool_pose)
    arm.gripper.open()
    arm.relative_move(2, 0.1)

def grasp_screw_tool(tool_pose, screw_direction):
    arm.move_to_neutral()
    offset_z = -0.2 if screw_direction else -0.175
    arm.move_to_joint(tool_pose)
    arm.gripper.open()
    arm.relative_move(2, offset_z)
    arm.gripper.move_joints(0.019)
    arm.relative_move(2, 0.1)

def place_screw_tool(tool_pose, screw_direction):
    arm.move_to_neutral()
    offset_z = -0.2 if screw_direction else -0.175
    arm.move_to_joint(pose_feature=tool_pose)
    arm.gripper.grasp(0.016, 20)
    arm.gripper.move_joints(0.019)
    arm.move_group.set_end_effector_link("panda_hand_tcp")
    arm.relative_move(2, offset_z)
    arm.gripper.open()
    arm.clear_error()
    arm.relative_move(2, 0.15)
    arm.move_to_neutral()

def set_force_contact_threshold(force_contact_threshold):
    if len(force_contact_threshold) != 6:
        print("force_contact_threshold must be a list of 6 elements")
        return
    arm.stop_controller(arm.controller_name)
    arm.lower_force = force_contact_threshold
    arm.upper_force = [max(x*2,18) for x in force_contact_threshold]
    arm.set_force_torque_collision_behavior(arm.lower_torque, arm.upper_torque, arm.lower_force, arm.upper_force)
    rprint(arm.lower_force)
    rprint(arm.upper_force)
    arm.start_controller(arm.controller_name)
    arm.clear_error()

def unscrew_screw(just_above_screw_pose, unscrew_time=2):
    arm.move_to_joint(just_above_screw_pose)
    arm.gripper.grasp(0.016, 20)
    arm.move_to_contact(speed=0.005)
    rprint("move_to_contact done")
    sleep(unscrew_time)
    rprint("sleep done")
    arm.relative_move(2, 0.02)
    rprint("relative_move done")
    arm.gripper.move_joints(0.019)
    arm.move_to_joint(pose_dict["approx_above_screen_with_screw_tool"])

def dropoff_screw_at_tool_holder(screw_hole_pose):
    arm.move_to_cartesian(screw_hole_pose)
    arm.gripper.grasp(0.016, 20)
    arm.move_to_contact(speed=0.005)
    arm.gripper.move_joints(0.019)
    arm.rotate(deg_to_rad(-10),0,0)
    current_pose = arm.get_current_pose()
    current_pose.position.y += 0.01
    current_pose.position.z += 0.01
    arm.move_to_cartesian(current_pose)
    arm.move_to_joint(pose_dict["approx_above_screen_with_screw_tool"])

def pick_up_screw_from_tool_holder(screw_hole_pose: Pose):
    arm.move_to_cartesian(screw_hole_pose)
    arm.gripper.grasp(0.016, 20)
    arm.move_to_contact(speed=0.005)
    arm.relative_move(2, 0.02)
    arm.gripper.move_joints(0.019)
    arm.move_to_joint(pose_dict["approx_above_screen_with_screw_tool"])

def unscrew_screws_from_pc():
    arm.move_to_joint(pose_dict["approx_above_screen_with_screw_tool"])
    old_threshold = arm.lower_force
    set_force_contact_threshold([5,5,5,10,10,10])

    unscrew_screw(pose_dict["just_above_screw1"])
    dropoff_screw_at_tool_holder(pose_dict["screw_hole1"])
    unscrew_screw(pose_dict["just_above_screw2"])
    dropoff_screw_at_tool_holder(pose_dict["screw_hole2"])
    unscrew_screw(pose_dict["just_above_screw3"])
    dropoff_screw_at_tool_holder(pose_dict["screw_hole3"])
    unscrew_screw(pose_dict["just_above_screw4"])
    dropoff_screw_at_tool_holder(pose_dict["screw_hole4"])

    arm.move_to_neutral()
    set_force_contact_threshold(old_threshold)


def generate_spiral(center_pose=None, radius_step=0.1, angle_step=0.1, num_points=10):
    points = []
    angle = 0
    radius = 0
    if center_pose is None:
        center_pose = arm.get_current_pose()
    
    for _ in range(num_points):
        x = center_pose.position.x + radius * cos(angle)
        y = center_pose.position.y + radius * sin(angle)
        points.append(Pose(position=Point(x=x, y=y, z=center_pose.position.z), orientation=center_pose.orientation))
        radius += radius_step
        angle += angle_step
    return points[1:]


def rescrew_screw(just_above_screw_pose, screw_time=2):
    arm.move_to_joint(just_above_screw_pose)
    arm.relative_move(2, -0.015)
    arm.gripper.grasp(0.016, 20)
    arm.move_to_contact(speed=0.005)
    sleep(screw_time)
    arm.relative_move(2, 0.02)
    arm.gripper.move_joints(0.019)
    arm.move_to_joint(pose_dict["approx_above_screen_with_screw_tool"])

def rescrew_screws_to_pc():
    arm.move_to_joint(pose_dict["approx_above_screen_with_screw_tool"])
    old_threshold = arm.lower_force
    set_force_contact_threshold([5,5,5,10,10,10])

    pick_up_screw_from_tool_holder(pose_dict["screw_hole4"])
    input("Press Enter to continue...")
    rescrew_screw(pose_dict["just_above_screw4"])
    pick_up_screw_from_tool_holder(pose_dict["screw_hole3"])
    input("Press Enter to continue...")
    rescrew_screw(pose_dict["just_above_screw3"])
    pick_up_screw_from_tool_holder(pose_dict["screw_hole2"])
    input("Press Enter to continue...")
    rescrew_screw(pose_dict["just_above_screw2"], screw_time=1.75)
    pick_up_screw_from_tool_holder(pose_dict["screw_hole1"])
    input("Press Enter to continue...")
    rescrew_screw(pose_dict["just_above_screw1"], screw_time=1.75)

    arm.move_to_neutral()
    set_force_contact_threshold(old_threshold)

def contact_with_screen_frame():
    arm.align_to_base()
    arm.rotate(0.1,0,0)
    result = arm.move_to_contact()
    if not result[0][2]:
        print("PANIC!!!")
        exit()  

    old_threshold = arm.lower_force
    set_force_contact_threshold([15.0, 15.0, 15.0, 18.0, 18.0, 18.0])
    current_pose = arm.get_current_pose()
    current_pose.position.y -= 0.1
    current_pose.position.z -= 0.002
    arm.move_to_contact(current_pose, only_in_axis=0)
    screen_frame = arm.get_current_pose()  
    set_force_contact_threshold(old_threshold)

def remove_screen_frame():
    current_pose = arm.get_current_pose()
    current_pose.position.z += 0.02
    arm.move_to_cartesian(current_pose)
    
    arm.relative_move(2, 0.02)
    arm.clear_error()
    current_pose = arm.get_current_pose()
    current_pose.position.z += 0.2
    current_pose.orientation = arm.rotate(-0.7,0,0, False).orientation
    arm.move_to_cartesian(current_pose)

def place_screen_frame_in_holder():
    arm.relative_move(2, 0.3)
    arm.rotate(np.pi/2,0,0)
    arm.move_to_joint(pose_dict["screen_frame_holder_pose"])

    pose = arm.get_current_pose()
    pose.position.y += 0.1
    arm.move_to_contact(pose)
    arm.relative_move(2, -0.06)
    arm.relative_move(1, -0.1)

def pickup_screen_frame_from_holder():
    arm.move_to_joint(pose_dict["screen_frame_holder_pose"])
    arm.move_to_cartesian(pose_dict["pickup_screen_frame_from_holder"])
    arm.gripper.grasp(0.035, 40)
    arm.relative_move(2,0.02)
    arm.relative_move(1,-0.2)

def replace_screen_frame():
    arm.rotate(-np.pi/2,0,0)
    arm.relative_move(1,-0.15)
    rotation_list = [arm.rotate(0,0,np.pi/2, False),arm.rotate(0,0,np.pi/2, False)]
    arm.move_to_cartesian(rotation_list)
    
    arm.move_to_cartesian(pose_dict["setpoint_replace_screen_frame"])
    arm.relative_move(2,-0.015)
    goal_pose = pose_dict["replace_screen_frame_pose"]
    goal_pose.position.y += 0.002
    arm.move_to_cartesian(goal_pose)
    arm.gripper.open()
    arm.relative_move(2,0.1)

def main():

    arm.clear_error()
    arm.move_to_neutral()
    arm.gripper.close()
    arm.set_speed(0.25)

    while True:
        arm.align_to_base(z=True)

        ### OPEN PC
        grasp_tool(pose_dict["pc_open_tool"])
        arm.move_group.set_end_effector_link("panda_tool_pc_open_tcp")
        arm.move_to_cartesian(pose_dict["approx_over_pc"])
        pc_closed_z = find_pc_z()
        # pc_closed_x = find_pc_x() NOT WORKING RIGHT NOW, TOO CLOSE TO JOINT LIMITS
        arm.relative_move(2, 0.02)
        arm.align_to_base()
        pc_closed_y = find_pc_y()
        pc_open_pose = open_pc(pc_closed_y, pc_closed_z)
        place_tool(tool_pose=pose_dict["pc_open_tool"])

        ### REMOVE SCREEN FRAME
        grasp_tool(pose_dict["screen_frame_remover_tool"])
        arm.move_group.set_end_effector_link("panda_tool_screen_frame_remover_tcp")
        arm.move_to_cartesian(pose_dict["approx_over_screen"])
        contact_with_screen_frame()
        remove_screen_frame()
        place_screen_frame_in_holder()
        arm.move_to_neutral()
        place_tool(pose_dict["screen_frame_remover_tool"])

        ### UNSCREW SCREWS
        grasp_screw_tool(pose_dict["approx_over_screw_tool"], screw_direction=0)
        arm.move_group.set_end_effector_link("panda_tool_unscrew_tcp")
        arm.move_to_neutral()
        unscrew_screws_from_pc()
        place_screw_tool(pose_dict["approx_over_screw_tool"], screw_direction=0)

        ### REMOVE SCREEN
        grasp_tool(pose_dict["screen_remover_tool"])
        arm.move_group.set_end_effector_link("panda_tool_screen_remover_tcp")
        dropoff_screen_pose = remove_screen()

        ### REATTACH SCREEN 
        pickup_screen_from_holder(dropoff_screen_pose)
        replace_screen()
        arm.move_to_neutral()
        place_tool(pose_dict["screen_remover_tool"])

        ### RESCREW SCREWS
        grasp_screw_tool(pose_dict["approx_over_screw_tool"], screw_direction=1)
        arm.move_group.set_end_effector_link("panda_tool_screw_tcp")
        arm.move_to_neutral()
        rescrew_screws_to_pc()
        place_screw_tool(pose_dict["approx_over_screw_tool"], screw_direction=1)

        ### REPLACE SCREEN FRAME
        pickup_screen_frame_from_holder()
        replace_screen_frame()
        arm.move_to_neutral()

        ### REATTACH SCREEN FRAME
        grasp_tool(pose_dict["pc_open_tool"])
        arm.move_group.set_end_effector_link("panda_tool_pc_open_tcp")
        arm.move_to_joint(pose_dict["above_screen_frame_reattach_startpoint"])
        reattach_screen_frame(force_z=8)
        arm.move_to_neutral()
        place_tool(pose_dict["pc_open_tool"])

        ### CLOSE PC
        grasp_tool(pose_dict["pc_open_tool"])
        arm.move_group.set_end_effector_link("panda_tool_pc_open_tcp")
        close_pc(pc_open_pose)
        place_tool(pose_dict["pc_open_tool"])
        
        arm.move_to_neutral()   
        break


if __name__ == "__main__":


    try:
        main()
    except rospy.ROSInterruptException:
        rprint("ROSInterruptException")
        arm.move_group.stop()
    except KeyboardInterrupt:
        rprint("KeyboardInterrupt")
        arm.move_group.stop()