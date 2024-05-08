from panda_ros_lib import PandaArm, IterativeParabolicTimeParameterization, rprint
import numpy as np
from copy import deepcopy
from geometry_msgs.msg import Pose, Point, Quaternion
from time import sleep
from math import dist, sin, cos
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
    "pc_open_tool": Pose(
        position=Point(
            x=0.05733242432911795, y=0.6689132445003902, z=0.05685740570286763 + 0.004
        ),
        orientation=Quaternion(
            x=0.999749275808511,
            y=-0.007967242863267691,
            z=0.007470049325425549,
            w=-0.019547555463311283,
        ),
    ),
    # screen_frame_remover_tool:
    # position:
    #     x: 0.11921723907546183
    #     y: 0.6682887607731034
    #     z: 0.05198510034165075
    # orientation:
    #     x: 0.9997503370695412
    #     y: -0.004974166528565403
    #     z: 0.0032409638284769306
    #     w: -0.021541061954963574
    "screen_frame_remover_tool": Pose(
        position=Point(
            x=0.11921723907546183, y=0.6682887607731034, z=0.05198510034165075
        ),
        orientation=Quaternion(
            x=0.9997503370695412,
            y=-0.004974166528565403,
            z=0.0032409638284769306,
            w=-0.021541061954963574,
        ),
    ),
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
    "reattach_screen_frame_tool": Pose(
        position=Point(
            x=0.16915693824357508, y=0.6683187553468177, z=0.061001380081872885
        ),
        orientation=Quaternion(
            x=0.9996913313731218,
            y=-0.014610600147982525,
            z=0.00577630464276479,
            w=-0.019245951403475937,
        ),
    ),
    # approx_over_pc:
    # position:
    #     x: 0.22201995933787994
    #     y: 0.12839919454023427
    #     z: 0.0430857855019747
    # orientation:
    #     x: 0.710601819071197
    #     y: -0.7035943573742
    #     z: 4.96265348249554e-07
    #     w: -0.00018709261380103005
    "approx_over_pc": Pose(
        position=Point(
            x=0.22201995933787994, y=0.12839919454023427, z=0.0430857855019747
        ),
        orientation=Quaternion(
            x=0.710601819071197,
            y=-0.7035943573742,
            z=4.96265348249554e-07,
            w=-0.00018709261380103005,
        ),
    ),
    # approx_over_screen:
    # position:
    #     x: 0.2007785586355047
    #     y: 0.3786916359605762
    #     z: 0.038138931532571134
    # orientation:
    #     x: 0.7127143494800567
    #     y: -0.7014080661131809
    #     z: 0.004175902436892146
    #     w: -0.006895119681679882
    "approx_over_screen": Pose(
        position=Point(
            x=0.2007785586355047, y=0.3786916359605762, z=0.038138931532571134
        ),
        orientation=Quaternion(
            x=0.7127143494800567,
            y=-0.7014080661131809,
            z=0.004175902436892146,
            w=-0.006895119681679882,
        ),
    ),
    # approx_over_screw_tool:
    # joint values: 1.4247089063927794, 0.02294454488027514, -0.33347968736871003, -2.613312880213784, -1.9871708795112801, 1.7851599069568844, -2.7739401184486407
    "approx_over_screw_tool": [
        1.4311203370395733,
        0.01589967568759577,
        -0.3352690462254318,
        -2.6189887238850136,
        -1.9814788819419014,
        1.7842299505207273,
        -2.7759469578915112,
    ],
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
    "above_screen_frame_reattach_startpoint": Pose(
        position=Point(
            x=0.37360320562584904, y=0.25827006719757184, z=0.049199265237291526
        ),
        orientation=Quaternion(
            x=-0.6984942505941352,
            y=0.7156153248200037,
            z=-0.0004737473805072651,
            w=0.0005141333660185798,
        ),
    ),
    "approx_above_screen_with_screw_tool": [
        1.8009205215855648,
        -0.9884424066794545,
        -1.8641107601701168,
        -2.083275166394418,
        0.7750173774303746,
        2.066743081026607,
        -1.4132072120328094,
    ],
    "just_above_screw1": [
        2.0473151920350166,
        -1.3172633878888524,
        -2.119343980808924,
        -1.5976601368470218,
        0.9184731412051637,
        2.363234942918682,
        -1.6017888109417486,
    ],
    "just_above_screw2": [
        1.7328304659692864,
        -1.0553802775332801,
        -2.2613807127861594,
        -1.9985897680379265,
        0.34595452260729637,
        1.6146659816899598,
        -1.5802690885332846,
    ],
    "just_above_screw3": [
        2.128083195463207,
        -1.086070538554275,
        -2.183045222198754,
        -2.045789285905178,
        0.4165805819564395,
        1.776043543711331,
        -1.5324558357381157,
    ],
    # if not new fixture with corners "just_above_screw3": [1.9068539003740277, -1.0985445564504255, -2.0992789021548406, -2.314217004943312, 0.5971005605599946, 2.1517024513218135, -1.651512649972728],
    "just_above_screw4": [
        2.496900336533262,
        -1.302216902849967,
        -2.249884076130255,
        -1.3326268258204164,
        0.8131843160523317,
        2.071208368553055,
        -1.4167155728340637,
    ],
    # if not new fixture with corners "just_above_screw4": [2.4193914513609065, -1.320201842140733, -2.1651112390362925, -1.4635632465885928, 0.8576647046496799, 2.2705184519496253, -1.499695961078008],
    # screw hole1
    # position:
    #     x: 0.17438816272508167
    #     y: 0.6883851077571157
    #     z: 0.03662979719443973
    # orientation:
    #     x: 0.6890184902575103
    #     y: 0.7247167265578114
    #     z: -0.005941444957152131
    #     w: 0.001971182998326503
    "screw_hole1": Pose(
        position=Point(
            x=0.17438816272508167, y=0.6883851077571157, z=0.03662979719443973
        ),
        orientation=Quaternion(
            x=0.6890184902575103,
            y=0.7247167265578114,
            z=-0.005941444957152131,
            w=0.001971182998326503,
        ),
    ),
    # screw hole2
    # position:
    #     x: 0.17482815627966847
    #     y: 0.673359298063665
    #     z: 0.035172471576562066
    # orientation:
    #     x: 0.6890236397156073
    #     y: 0.7247052081531279
    #     z: -0.006777953019886442
    #     w: 0.0016865769028615435
    "screw_hole2": Pose(
        position=Point(
            x=0.17482815627966847, y=0.673359298063665, z=0.035172471576562066
        ),
        orientation=Quaternion(
            x=0.6890236397156073,
            y=0.7247052081531279,
            z=-0.006777953019886442,
            w=0.0016865769028615435,
        ),
    ),
    # screw hole3
    # position:
    #     x: 0.1664217456230898
    #     y: 0.6582952010929098
    #     z: 0.036670562939416854
    # orientation:
    #     x: 0.6892174154108944
    #     y: 0.7244904235665832
    #     z: -0.009584289267318132
    #     w: 0.0010591760382997403
    "screw_hole3": Pose(
        position=Point(
            x=0.1664217456230898, y=0.6582952010929098, z=0.036670562939416854
        ),
        orientation=Quaternion(
            x=0.6892174154108944,
            y=0.7244904235665832,
            z=-0.009584289267318132,
            w=0.0010591760382997403,
        ),
    ),
    # screw hole4
    # position:
    #     x: 0.17532651654498604
    #     y: 0.6433158715508616
    #     z: 0.034407648468695075
    # orientation:
    #     x: 0.6896277063275514
    #     y: 0.7241284265106904
    #     z: -0.006982747151829334
    #     w: 0.0016999490480331667
    "screw_hole4": Pose(
        position=Point(
            x=0.17532651654498604, y=0.6433158715508616, z=0.034407648468695075
        ),
        orientation=Quaternion(
            x=0.6896277063275514,
            y=0.7241284265106904,
            z=-0.006982747151829334,
            w=0.0016999490480331667,
        ),
    ),
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
    "pc_open_pose": Pose(
        position=Point(
            x=0.22488338974878735, y=0.5337942703884697, z=0.012429523185995345
        ),
        orientation=Quaternion(
            x=0.7115514192389913,
            y=-0.7026241814753624,
            z=0.0017259332275041644,
            w=-0.0032952298172670823,
        ),
    ),
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
    "screen_remover_tool": Pose(
        position=Point(
            x=-0.09752514464401774, y=0.6699170123947952, z=0.052242802087636785
        ),
        orientation=Quaternion(
            x=0.9998842004260421,
            y=0.011519073565381807,
            z=-0.0011068183837063593,
            w=-0.00988289611579314,
        ),
    ),
    # screen_remover_starting_point
    # position:
    #     x: 0.10307521912937706
    #     y: 0.37181146502201834
    #     z: 0.4777009669742028
    # orientation:
    #     x: 0.9998724064556803
    #     y: 0.011885856262552853
    #     z: -0.00046915861588556424
    #     w: -0.01066194727177901
    "screen_remover_starting_point": Pose(
        position=Point(
            x=0.10307521912937706, y=0.37181146502201834, z=0.4777009669742028
        ),
        orientation=Quaternion(
            x=0.9998724064556803,
            y=0.011885856262552853,
            z=-0.00046915861588556424,
            w=-0.01066194727177901,
        ),
    ),
    "screen_remover_init_pose": [
        1.2586937529445135,
        1.141014569198876,
        -1.5285425722009187,
        -1.963085132983693,
        1.1754174835417006,
        1.8310459912037844,
        1.7666723326858293,
    ],
    "screen_frame_holder_pose": [
        -0.3372660331767902,
        0.908194028473517,
        0.7066221270561218,
        -1.2979891606535983,
        1.1430772891574437,
        1.9559269683123433,
        -1.2271082312517112,
    ],
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
    "replace_screen_frame_pose": Pose(
        position=Point(
            x=0.20760234949738338, y=0.2666872894619938, z=0.021453934618378794
        ),
        orientation=Quaternion(
            x=0.00890364091094405,
            y=0.9998229758075018,
            z=-0.016138900135312096,
            w=0.0037786410762532174,
        ),
    ),
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
    "setpoint_replace_screen_frame": Pose(
        position=Point(
            x=0.20539675969660026, y=0.16133011533858677, z=0.10627113002249214
        ),
        orientation=Quaternion(
            x=0.010050573066068973,
            y=0.9998044633837311,
            z=-0.016748926566094887,
            w=0.0030813045771607393,
        ),
    ),
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
    "pickup_screen_frame_from_holder": Pose(
        position=Point(
            x=0.3505188466055554, y=0.7079726656182903, z=0.2872879247477031
        ),
        orientation=Quaternion(
            x=-0.6955171374323214,
            y=0.007073613147483974,
            z=0.014305651282145912,
            w=0.7183322517306203,
        ),
    ),
}

tool_weights = {
    "panda_tool_pc_open": 0.014,
    "panda_tool_frame_remover": 0.018,
    "screen_frame": 0.03,
    "panda_tool_screen_remover": 0.088,
    "panda_tool_screwdriver": 0.072,
    "pc_screen": 0.34,
    "realsense_camera": 0.072,
}


def deg_to_rad(deg):
    return deg * (np.pi / 180)


def rad_to_deg(rad):
    return rad * (180 / np.pi)


def find_pc_z():
    arm.align_to_base()
    arm.move_to_contact(force_threshold=2.5)
    current_pose = arm.get_current_pose()
    pc_z = current_pose.position.z
    return pc_z


def find_pc_x():
    arm.relative_move(2, 0.01)
    arm.relative_move(0, -0.25)
    arm.relative_move(2, -0.04)
    pose = arm.get_current_pose()
    pose.position.x += 0.3
    arm.move_to_contact(pose)

    current_pose = arm.get_current_pose()
    pc_x_min = current_pose.position.x

    arm.relative_move(2, 0.04)
    arm.relative_move(0, 0.45)
    arm.rotate(0, 0, np.pi)
    arm.relative_move(2, -0.04)
    pose = arm.get_current_pose()
    pose.position.x -= 0.3
    arm.move_to_contact(pose)

    current_pose = arm.get_current_pose()
    pc_x_max = current_pose.position.x

    pc_x_center = (pc_x_min + pc_x_max) / 2

    arm.relative_move(2, 0.04)
    current_pose = arm.get_current_pose()
    current_pose.position.x = pc_x_center
    arm.move_to_cartesian(current_pose)
    arm.rotate(0, 0, np.pi / 2)

    return {"center": pc_x_center, "min": pc_x_min, "max": pc_x_max}


def find_pc_y():
    arm.relative_move(1, -0.15)
    arm.relative_move(2, -0.04)
    pose = arm.get_current_pose()
    pose.position.y += 0.3
    arm.move_to_contact(pose, force_threshold=5)

    current_pose = arm.get_current_pose()
    pc_y = current_pose.position.y
    return pc_y


def open_pc(pc_closed_y, pc_closed_z):
    arm.relative_move(1, -0.03)
    #
    move_list = [arm.rotate(0, 0, np.pi / 2, False), arm.rotate(0, 0, np.pi, False)]
    arm.move_to_cartesian(move_list)
    #
    pc_lid_thickness = 0.007
    current_pose = arm.get_current_pose()
    current_pose.position.z = pc_closed_z - pc_lid_thickness - 0.004
    arm.move_to_cartesian(current_pose)
    #
    current_pose = arm.get_current_pose()
    current_pose.position.y = pc_closed_y
    arm.move_to_cartesian(current_pose)
    #
    current_pose = arm.get_current_pose()
    current_pose.position.y = 0.05
    arm.move_to_contact(current_pose, force_threshold=5)
    #
    arm.move_group.set_end_effector_link("panda_pc_open_tcp")
    arm.rotate(-np.pi / 2, 0, 0)
    #
    arm.move_group.set_end_effector_link("panda_tool_pc_open_tcp")
    arm.relative_move(2, 0.1)
    #
    current_pose = arm.get_current_pose()
    rotation_list = [
        arm.rotate(np.pi / 2, 0, 0, False),
        arm.rotate(np.pi, np.pi / 2, 0, False),
        arm.rotate(np.pi, np.pi, 0, False),
    ]
    arm.move_to_cartesian(rotation_list)
    #
    arm.relative_move(1, -0.02)
    arm.relative_move(2, -0.11)
    #
    arm.move_group.set_end_effector_link("panda_pc_open_tcp")
    arm.rotate(-np.pi / 2 + deg_to_rad(15), 0, 0)
    arm.move_group.set_end_effector_link("panda_tool_pc_open_tcp")
    arm.align_to_base()
    arm.move_to_contact(force_threshold=2.5)
    arm.relative_move(2, 0.02)
    return arm.get_current_pose()


def close_pc(pc_open_pose):
    target_pose = deepcopy(pc_open_pose)
    target_pose.position.y += 0.08
    target_pose.position.z -= 0.025
    arm.move_to_cartesian(target_pose)
    current_pose = arm.get_current_pose()
    current_pose.position.y -= 0.05
    arm.move_to_contact(current_pose, force_threshold=8)
    arm.move_group.set_end_effector_link("panda_pc_open_tcp")
    arm.rotate(np.pi / 2, 0, 0)
    #
    arm.move_group.set_end_effector_link("panda_tool_pc_open_tcp")
    arm.relative_move(2, 0.1)
    current_pose = arm.get_current_pose()
    rotation_lst = [
        arm.rotate(-np.pi / 2, 0, 0, False),
        arm.rotate(-np.pi, np.pi / 2, 0, False),
        arm.rotate(-np.pi, np.pi, 0, False),
    ]
    arm.move_to_cartesian(rotation_lst)
    arm.relative_move(1, 0.02)
    arm.relative_move(2, -0.11)
    #
    arm.move_group.set_end_effector_link("panda_pc_open_tcp")
    arm.rotate(np.pi / 2 - deg_to_rad(10), 0, 0)
    arm.move_group.set_end_effector_link("panda_tool_pc_open_tcp")
    arm.align_to_base()
    arm.relative_move(2, 0.02)


def wait_for_move_complete(goal_pose, tolerance=0.005):
    current_pose = arm.get_current_pose()
    d = dist(
        [current_pose.position.x, current_pose.position.y, current_pose.position.z],
        [goal_pose.position.x, goal_pose.position.y, goal_pose.position.z],
    )
    while d > tolerance:
        sleep(1)
        current_pose = arm.get_current_pose()
        d = dist(
            [current_pose.position.x, current_pose.position.y, current_pose.position.z],
            [goal_pose.position.x, goal_pose.position.y, goal_pose.position.z],
        )
        rprint(d)


def reattach_screen_frame(force_z):
    rprint("start reattach_screen_frame")
    arm.start_cartestion_impedance_controller(
        [1000, 1000, 1000, 10, 10, 10], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    )
    arm.move_to_contact(speed=0.005, force_threshold=5)
    rprint("set_cartestion_impedance_wrench")
    arm.set_cartestion_impedance_wrench([0, 0, force_z], [0, 0, 0])
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
    wait_for_move_complete(goal_pose=goal_pose, tolerance=0.017)
    rprint("fourth move")
    goal_pose = arm.relative_move(1, -0.22)
    wait_for_move_complete(goal_pose=goal_pose, tolerance=0.01)
    rprint("stop_cartestion_impedance_controller")
    arm.set_cartestion_impedance_wrench([0, 0, 0], [0, 0, 0])
    arm.stop_cartestion_impedance_controller()
    arm.relative_move(2, 0.02)


def reattach_screen_frame_NEW(force_z):
    rprint("start reattach_screen_frame")
    arm.start_cartestion_impedance_controller(
        [1000, 1000, 1000, 10, 10, 10], [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    )
    arm.move_to_contact(speed=0.005, force_threshold=5)
    rprint("set_cartestion_impedance_wrench")
    arm.set_cartestion_impedance_wrench([0, 0, force_z], [0, 0, 0])
    sleep(1)
    rprint("first move")
    goal_pose = arm.relative_move(0, -0.365)
    wait_for_move_complete(goal_pose=goal_pose, tolerance=0.01)
    rprint("second move part 1, lige frem")
    goal_pose = arm.relative_move(1, 0.17)
    wait_for_move_complete(goal_pose=goal_pose, tolerance=0.01)
    rprint("second move part 2, indrykning til højre")
    goal_pose = arm.get_current_pose()
    goal_pose.position.x += 0.015
    goal_pose.position.y += 0.075
    arm.move_to_cartesian(goal_pose)
    wait_for_move_complete(goal_pose=goal_pose, tolerance=0.01)
    rprint("third move part 1, lige frem")
    goal_pose = arm.relative_move(0, 0.06)
    wait_for_move_complete(goal_pose=goal_pose, tolerance=0.01)
    rprint("third move part 2, udrykning til venstre")
    goal_pose = arm.get_current_pose()
    goal_pose.position.x += 0.23
    goal_pose.position.y += 0.01
    arm.move_to_cartesian(goal_pose)
    wait_for_move_complete(goal_pose=goal_pose, tolerance=0.01)
    rprint("third move part 3, indrykning til højre")
    goal_pose = arm.get_current_pose()
    goal_pose.position.x += 0.075
    goal_pose.position.y -= 0.025
    arm.move_to_cartesian(goal_pose)
    wait_for_move_complete(goal_pose=goal_pose, tolerance=0.01)
    rprint("fourth move part 1, lige frem")
    goal_pose = arm.relative_move(1, -0.05)
    wait_for_move_complete(goal_pose=goal_pose, tolerance=0.01)
    rprint("fourth move part 2, udrykning til venstre")
    goal_pose = arm.get_current_pose()
    goal_pose.position.x += 0.015
    goal_pose.position.y -= 0.18
    arm.move_to_cartesian(goal_pose)
    wait_for_move_complete(goal_pose=goal_pose, tolerance=0.01)
    rprint("stop_cartestion_impedance_controller")
    arm.set_cartestion_impedance_wrench([0, 0, 0], [0, 0, 0])
    arm.stop_cartestion_impedance_controller()
    arm.relative_move(2, 0.02)


def remove_screen():
    # TODO: lav til fix position
    # current_pose = arm.get_current_pose()
    # current_pose.position.z += 0.35
    # current_pose.position.y -= 0.3
    # current_pose.position.x += 0.2
    # arm.move_to_cartesian(current_pose)
    arm.move_to_cartesian(pose_dict["screen_remover_starting_point"])
    arm.move_to_joint(pose_dict["screen_remover_init_pose"])
    arm.move_to_cartesian(pose_dict["approx_over_screen"])
    arm.rotate(deg_to_rad(2.2), 0, 0)
    # old_threshold = arm.lower_force
    # set_force_contact_threshold([20, 20, 20, 23, 23, 23])
    arm.move_to_contact(force_threshold=20)
    sleep(1)
    arm.relative_move(2, 0.05)
    #
    set_robot_load(
        tool_weights["panda_tool_screen_remover"] + tool_weights["pc_screen"]
    )
    #
    current_joint_pose = list(arm.state.q)
    current_joint_pose[6] -= np.pi
    arm.move_to_joint(current_joint_pose)
    current_pose = arm.get_current_pose()
    current_pose.position.x -= 0.41
    current_pose.position.y -= 0.33
    arm.move_to_cartesian(current_pose)
    #
    # set_force_contact_threshold(old_threshold)
    arm.relative_move(2, -0.05)
    # consider if it needs to align to base
    arm.rotate(deg_to_rad(-2.2), 0, 0)
    arm.move_to_contact(force_threshold=5)
    current_pose = arm.get_current_pose()
    current_pose.position.y -= 0.03
    arm.move_to_contact(current_pose, force_threshold=5)
    arm.clear_error()
    current_pose = arm.get_current_pose()
    current_pose.position.x += 0.02
    current_pose.position.y -= 0.0025
    arm.move_to_contact(current_pose, force_threshold=5)
    dropoff_screen_pose = arm.get_current_pose()
    # set_force_contact_threshold(force_contact_threshold=old_threshold)
    arm.clear_error()
    #
    arm.move_group.set_end_effector_link("panda_tool_screen_remover_tcp_off")
    arm.rotate(deg_to_rad(-10), 0, 0)
    arm.relative_move(2, 0.05)
    arm.move_group.set_end_effector_link("panda_tool_screen_remover_tcp")
    #
    set_robot_load(tool_weights["panda_tool_screen_remover"])
    return dropoff_screen_pose


def pickup_screen_from_holder(dropoff_screen_pose):
    dropoff_screen_pose_above = deepcopy(dropoff_screen_pose)
    dropoff_screen_pose_above.position.z += 0.01
    arm.move_to_cartesian(dropoff_screen_pose_above)
    old_threshold = arm.lower_force
    set_force_contact_threshold([35, 35, 35, 38, 38, 38])
    # check if we align with table before move to contact
    # arm.rotate(-0.02, 0, 0)
    # arm.rotate(0, 0.02, 0)
    arm.align_to_base()
    arm.move_to_contact(force_threshold=25)
    current_pose = arm.get_current_pose()
    current_pose.position.x -= 0.01
    current_pose.position.y += 0.04
    arm.move_to_cartesian(current_pose)
    arm.relative_move(2, 0.1)
    set_force_contact_threshold(old_threshold)
    set_robot_load(
        tool_weights["panda_tool_screen_remover"] + tool_weights["pc_screen"]
    )


def replace_screen():
    current_joint_pose = list(arm.state.q)
    current_joint_pose[6] += np.pi
    arm.move_to_joint(current_joint_pose)
    arm.move_to_cartesian(pose_dict["approx_over_screen"])
    arm.relative_move(0, -0.0015)
    arm.move_to_contact(force_threshold=8)
    rprint(arm.robot_mode)
    arm.clear_error()
    # old_threshold = arm.lower_force
    # set_force_contact_threshold([15, 15, 15, 18, 18, 18])
    arm.move_group.set_end_effector_link("panda_tool_screen_remover_tcp_off")
    arm.rotate(deg_to_rad(10), 0, 0)
    arm.relative_move(2, 0.1)
    arm.move_group.set_end_effector_link("panda_tool_screen_remover_tcp")
    # set_force_contact_threshold(old_threshold)
    set_robot_load(tool_weights["panda_tool_screen_remover"])


def grasp_tool(tool_pose, end_effector_link, tool_weight):
    tool_pose_above = deepcopy(tool_pose)
    tool_pose_above.position.z += 0.1
    arm.move_to_joint(tool_pose_above)
    arm.gripper.open()

    arm.move_to_cartesian(tool_pose)
    arm.gripper.grasp(0.02, 40)
    arm.relative_move(2, 0.1)

    change_tool(end_effector_link, tool_weight)


def change_tool(end_effector_link, tool_weight):
    arm.stop_controller(arm.controller_name)

    arm.move_group.set_end_effector_link(end_effector_link)
    arm.set_EE_frame([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
    arm.set_load(
        tool_weight + tool_weights["realsense_camera"],
        [0, 0, 0],
        load_inertia=[0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001],
    )

    arm.start_controller(arm.controller_name)
    arm.clear_error()


def set_robot_load(tool_weight):
    arm.stop_controller(arm.controller_name)
    arm.set_load(
        tool_weight, [0, 0, 0], load_inertia=[0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
    )
    arm.start_controller(arm.controller_name)


def place_tool(tool_pose):
    rprint("place_tool")
    tool_pose_above = deepcopy(tool_pose)
    tool_pose_above.position.z += 0.1
    rprint("move_to_joint")
    arm.move_to_joint(tool_pose_above)
    rprint("move_to_joint done")
    arm.move_group.set_end_effector_link("panda_hand_tcp")
    #
    arm.move_to_cartesian(tool_pose)
    arm.gripper.open()
    arm.relative_move(2, 0.1)
    #
    change_tool("panda_hand_tcp", 0.0)


def grasp_screw_tool(tool_pose, screw_direction):
    arm.move_to_neutral()
    offset_z = -0.2 if screw_direction else -0.175
    arm.move_to_joint(tool_pose)
    arm.gripper.open()
    arm.relative_move(2, offset_z)
    # arm.gripper.move_joints(0.019)
    arm.gripper.move_joints(0.016)
    arm.relative_move(2, 0.1)

    arm.stop_controller(arm.controller_name)

    arm.move_group.set_end_effector_link("panda_tool_unscrew_tcp")
    arm.set_EE_frame(
        [
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            -1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            -0.088,
            1.0,
        ]
    )
    arm.set_load(
        tool_weights["panda_tool_screwdriver"] + tool_weights["realsense_camera"],
        [0, 0, 0],
        load_inertia=[0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001],
    )

    arm.start_controller(arm.controller_name)
    arm.clear_error()


def place_screw_tool(tool_pose, screw_direction):
    arm.move_to_neutral()
    offset_z = -0.2 if screw_direction else -0.175
    arm.move_to_joint(pose_feature=tool_pose)
    arm.gripper.grasp(0.015, 20)
    arm.gripper.move_joints(0.016)
    arm.move_group.set_end_effector_link("panda_hand_tcp")
    arm.relative_move(2, offset_z)
    arm.gripper.open()
    arm.clear_error()
    arm.relative_move(2, 0.15)
    arm.move_to_neutral()

    change_tool("panda_hand_tcp", 0.0)


def set_force_contact_threshold(force_contact_threshold):
    if len(force_contact_threshold) != 6:
        print("force_contact_threshold must be a list of 6 elements")
        return
    arm.stop_controller(arm.controller_name)
    arm.lower_force = force_contact_threshold
    arm.upper_force = [max(x * 2, 18) for x in force_contact_threshold]
    arm.set_force_torque_collision_behavior(
        arm.lower_torque, arm.upper_torque, arm.lower_force, arm.upper_force
    )
    rprint(arm.lower_force)
    rprint(arm.upper_force)
    arm.start_controller(arm.controller_name)
    arm.clear_error()


def unscrew_screw(
    just_above_screw_pose,
    unscrew_time=2,
    on_screw_threshold=0.0023,
    in_screw_distance=0.003,
):
    arm.move_to_joint(just_above_screw_pose)
    arm.move_to_contact(speed=0.005, force_threshold=2.5)
    rprint(msg="move_to_contact done")
    # check if the screwdriver is on the screw head
    on_screw_head = check_if_on_screw_head(on_screw_threshold=on_screw_threshold)
    # if on_screw_head:
    #     spiral_search(speed=0.002)
    #     arm.move_to_contact(speed=0.005)
    # check if the screwdriver is in the screw head
    in_screw_head = check_if_in_screw_head(in_screw_distance, speed=0.005)
    if in_screw_head:
        # Vinkel ret på planet
        arm.rotate(deg_to_rad(2.8), 0, 0)
        rprint(arm.robot_mode)
        arm.gripper.grasp(0.015, 20)
        sleep(1.7)
        rprint("sleep done")
        # arm.relative_move(2, 0.0035, speed=0.003)
        # arm.align_to_base()
        # arm.relative_move(2, 0.02)
        # rprint("relative_move done")
        p = arm.get_current_pose()
        p.position.z += 0.02
        p.position.y -= 0.003
        # arm.relative_move(2, 0.02)
        arm.move_to_cartesian(p)
        arm.gripper.move_joints(0.016)
        arm.move_to_joint(pose_dict["approx_above_screen_with_screw_tool"])
    else:
        unscrew_screw(just_above_screw_pose, unscrew_time=unscrew_time)


# def unscrew_screw(just_above_screw_pose, unscrew_time=2):
#     p1 = deepcopy(just_above_screw_pose)
#     p1.position.z += 0.003
#     arm.move_to_cartesian(p1)
#     arm.gripper.grasp(0.015, 20)
#     arm.move_to_cartesian(just_above_screw_pose)
#     arm.move_to_contact(speed=0.005)
#     rprint(msg="move_to_contact done")
#     # spiral_points = generate_spiral(center_pose=arm.get_current_pose(), radius_step=0.00002, angle_step=0.1, num_points=100)
#     # arm.move_to_cartesian(pose_feature=spiral_points, speed=0.005)
#     # arm.relative_move(2,0.0005)
#     spiral_search()
#     # arm.move_to_contact(speed=0.005)
#     # arm.rotate(deg_to_rad(2.8),0,0)
#     rprint(arm.robot_mode)
#     # arm.gripper.grasp(0.015, 20)
#     sleep(1.7)
#     rprint("sleep done")
#     # arm.relative_move(2, 0.0035, speed=0.003)
#     # arm.align_to_base()
#     # arm.relative_move(2, 0.02)
#     # rprint("relative_move done")
#     p = arm.get_current_pose()
#     p.position.z += 0.02
#     p.position.y -= 0.003
#     # arm.relative_move(2, 0.02)
#     arm.move_to_cartesian(p)
#     arm.gripper.move_joints(0.016)
#     arm.move_to_joint(pose_dict["approx_above_screen_with_screw_tool"])


def spiral_search(
    radius_step=0.00002,
    angle_step=0.1,
    num_points=100,
    speed=0.005,
    calibrate_force_sensor=True,
):
    spiral_points = generate_spiral(
        center_pose=arm.get_current_pose(),
        radius_step=radius_step,
        angle_step=angle_step,
        num_points=num_points,
    )
    if calibrate_force_sensor:
        arm.zero_force_sensor()
    calibrated_force = [
        arm.force_x_avg - arm.calibrate_force_sensor[0],
        arm.force_y_avg - arm.calibrate_force_sensor[1],
        arm.force_z_avg - arm.calibrate_force_sensor[2],
    ]
    arm.move_to_cartesian(spiral_points, speed=speed, make_interuptable=False)
    try:
        rprint(arm.position_trajectory_status)
        while (
            arm.position_trajectory_status is None
            or arm.position_trajectory_status != 1
        ):
            # wait for the robot to start moving
            sleep(0.1)
        while arm.position_trajectory_status != 3:
            # loop until the robot is done moving
            # if arm.contact_state[1] == 1.0 or arm.contact_state[2] == 1.0:
            calibrated_force = [
                arm.force_x_avg - arm.calibrate_force_sensor[0],
                arm.force_y_avg - arm.calibrate_force_sensor[1],
                arm.force_z_avg - arm.calibrate_force_sensor[2],
            ]
            rprint(calibrated_force)
            # DO: ændre lower force til input force og tjek på længden af kraftvektoren
            if (abs(calibrated_force[0]) > arm.lower_force[0]) or (
                abs(calibrated_force[1]) > arm.lower_force[1]
            ):
                # TODO: use force calibrated sensor value
                rprint("contact detected")
                rprint(arm.contact_state)
                arm.move_group.stop()
                break
            sleep(0.1)
        arm.position_trajectory_status = None
        sleep(0.1)
    except KeyboardInterrupt:
        arm.move_group.stop()
        arm.clear_error()


def check_if_on_screw_head(on_screw_threshold=0.0023, do_spiral_search=True):
    # over threshold then on screw head else in or beside screw
    rprint("check_if_on_screw_head")
    rprint(arm.get_current_pose().position.z)
    if arm.get_current_pose().position.z > on_screw_threshold:
        rprint("on screw head")
        if do_spiral_search:
            spiral_search(speed=0.002)
            # TODO: Make force a parameter to check
            arm.move_to_contact(speed=0.005, force_threshold=2.5)
            check_if_on_screw_head(on_screw_threshold)
        else:
            return True
    else:
        return False


def check_if_in_screw_head(contact_distance=0.003, speed=0.005):
    # check if contact with one side
    rprint("check_if_in_screw_head")
    # first_side_contact = False
    # second_side_contact = False
    # arm.relative_move(0,contact_distance, speed=speed)
    start_pose = arm.get_current_pose()
    first_side_pose = arm.get_current_pose()
    first_side_pose.position.x += contact_distance
    first_side_contact = arm.move_to_contact(
        first_side_pose, speed=speed, calibrate_force_sensor=True, force_threshold=2.5
    )
    # arm.move_to_cartesian(start_pose, speed=speed)
    # if arm.contact_state[1] == 1.0 or arm.contact_state[2] == 1.0:
    #     first_side_contact = True
    # else:
    #     first_side_contact = False
    # check if contact with other side
    # arm.relative_move(0,distance=-contact_distance*2, speed=speed)
    second_side_pose = arm.get_current_pose()
    second_side_pose.position.x -= contact_distance * 2
    second_side_contact = arm.move_to_contact(
        second_side_pose, speed=speed, calibrate_force_sensor=True, force_threshold=2.5
    )
    # if arm.contact_state[1] == 1.0 or arm.contact_state[2] == 1.0:
    #     second_side_contact = True
    # else:
    #     second_side_contact = False
    # move back to original position
    # arm.relative_move(0,contact_distance, speed=speed)
    # first_side_pose.position.x -= contact_distance
    arm.move_to_cartesian(start_pose, speed=speed)
    # if contact with both sides then in screw head else beside screw head
    if first_side_contact and second_side_contact:
        return True
    else:
        return False


def dropoff_screw_at_tool_holder(screw_hole_pose):
    arm.move_to_cartesian(screw_hole_pose)
    arm.gripper.grasp(0.015, 20)
    arm.move_to_contact(speed=0.005, force_threshold=2.5)
    # arm.gripper.move_joints(0.019)
    arm.gripper.move_joints(0.016)
    arm.rotate(deg_to_rad(-10), 0, 0)
    current_pose = arm.get_current_pose()
    current_pose.position.y += 0.01
    current_pose.position.z += 0.01
    arm.move_to_cartesian(current_pose)
    arm.move_to_joint(pose_dict["approx_above_screen_with_screw_tool"])


def pick_up_screw_from_tool_holder(screw_hole_pose: Pose):
    arm.move_to_cartesian(screw_hole_pose)
    arm.gripper.grasp(0.015, 20)
    arm.move_to_contact(speed=0.005, force_threshold=2.5)
    # arm.relative_move(0,0.001)
    # arm.relative_move(1,-0.001)
    # arm.relative_move(0,-0.001)
    # arm.relative_move(1,0.001)
    # arm.gripper.grasp(0.015, 20)
    # arm.align_to_base()
    arm.gripper.move_joints(0.016)
    arm.relative_move(2, 0.02)
    arm.move_to_joint(pose_dict["approx_above_screen_with_screw_tool"])


def unscrew_screws_from_pc():
    arm.move_to_joint(pose_feature=pose_dict["approx_above_screen_with_screw_tool"])
    # old_threshold = arm.lower_force
    # set_force_contact_threshold([2.5, 2.5, 2.5, 10, 10, 10])
    # set_force_contact_threshold([5, 5, 5, 10, 10, 10])
    unscrew_screw(
        [
            2.0487224914316546,
            -1.3173170920648096,
            -2.117981006956937,
            -1.595953233702141,
            0.9182115273979867,
            2.3640687603696358,
            -1.6013832015170881,
        ]
    )  # pose_dict["just_above_screw1"])
    dropoff_screw_at_tool_holder(pose_dict["screw_hole1"])
    unscrew_screw(pose_dict["just_above_screw2"], on_screw_threshold=0.0)
    dropoff_screw_at_tool_holder(pose_dict["screw_hole2"])
    unscrew_screw(pose_dict["just_above_screw3"], on_screw_threshold=0.011)
    dropoff_screw_at_tool_holder(pose_dict["screw_hole3"])
    unscrew_screw(
        [
            2.5038271004985266,
            -1.3077519606540076,
            -2.2509133312827663,
            -1.3208191115396233,
            0.8100252142482334,
            2.0674106085977897,
            -1.4130818641185678,
        ],
        on_screw_threshold=0.011,
    )
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
        points.append(
            Pose(
                position=Point(x=x, y=y, z=center_pose.position.z),
                orientation=center_pose.orientation,
            )
        )
        radius += radius_step
        angle += angle_step
    return points[1:]


def rescrew_screw(just_above_screw_pose, screw_time=2):
    arm.move_to_joint(just_above_screw_pose)
    # arm.relative_move(2, -0.015)
    arm.gripper.grasp(0.015, 20)
    arm.move_to_contact(speed=0.005, force_threshold=2.5)
    # arm.relative_move(0,0.001)
    # arm.relative_move(1,-0.001)
    # arm.relative_move(0,-0.001)
    # arm.relative_move(1,0.001)
    # arm.align_to_base()
    # arm.gripper.grasp(0.015, 20)
    # arm.relative_move(2, -0.001)
    sleep(screw_time)
    arm.gripper.move_joints(0.016)
    arm.relative_move(2, 0.02)
    arm.move_to_joint(pose_dict["approx_above_screen_with_screw_tool"])


def rescrew_screws_to_pc():
    arm.move_to_joint(pose_dict["approx_above_screen_with_screw_tool"])
    # old_threshold = arm.lower_force
    # set_force_contact_threshold([5, 5, 5, 10, 10, 10])
    #
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
    #
    arm.move_to_neutral()
    # set_force_contact_threshold(old_threshold)


def contact_with_screen_frame():
    arm.align_to_base()
    arm.rotate(0.1, 0, 0)
    arm.move_to_contact(force_threshold=5)
    # old_threshold = arm.lower_force
    # set_force_contact_threshold([15.0, 15.0, 15.0, 18.0, 18.0, 18.0])
    current_pose = arm.get_current_pose()
    current_pose.position.y -= 0.1
    current_pose.position.z -= 0.005
    arm.move_to_contact(current_pose, only_in_axis=0, force_threshold=8)
    screen_frame = arm.get_current_pose()
    # set_force_contact_threshold(old_threshold)


def remove_screen_frame():
    current_pose = arm.get_current_pose()
    current_pose.position.z += 0.02
    arm.move_to_cartesian(current_pose)

    arm.relative_move(2, 0.02)
    arm.clear_error()
    current_pose = arm.get_current_pose()
    current_pose.position.z += 0.2
    current_pose.orientation = arm.rotate(-0.7, 0, 0, False).orientation
    arm.move_to_cartesian(current_pose)
    set_robot_load(
        tool_weights["panda_tool_frame_remover"] + tool_weights["screen_frame"]
    )


def place_screen_frame_in_holder():
    arm.relative_move(2, 0.3)
    arm.rotate(np.pi / 2, 0, 0)
    arm.move_to_joint(pose_dict["screen_frame_holder_pose"])
    pose = arm.get_current_pose()
    pose.position.y += 0.1
    arm.move_to_contact(pose, force_threshold=5)
    arm.relative_move(2, -0.06)
    arm.relative_move(1, -0.1)
    set_robot_load(tool_weights["panda_tool_frame_remover"])


def pickup_screen_frame_from_holder():
    arm.move_to_joint(pose_dict["screen_frame_holder_pose"])
    arm.move_to_cartesian(pose_dict["pickup_screen_frame_from_holder"])
    arm.gripper.grasp(0.035, 20)
    arm.relative_move(2, 0.02)
    arm.relative_move(1, -0.2)
    set_robot_load(
        tool_weights["panda_tool_frame_remover"] + tool_weights["screen_frame"]
    )


def replace_screen_frame(goal_pose_offset=0.003):
    arm.rotate(-np.pi / 2, 0, 0)
    arm.relative_move(1, -0.15)
    rotation_list = [
        arm.rotate(0, 0, np.pi / 2, False),
        arm.rotate(0, 0, np.pi / 2, False),
    ]
    arm.move_to_cartesian(rotation_list)

    arm.move_to_cartesian(pose_dict["setpoint_replace_screen_frame"])
    arm.relative_move(2, -0.015)
    goal_pose = deepcopy(pose_dict["replace_screen_frame_pose"])
    goal_pose.position.y += goal_pose_offset
    arm.move_to_cartesian(goal_pose)
    arm.gripper.open()
    arm.relative_move(2, 0.1)
    set_robot_load(0)


def main():

    arm.clear_error()
    arm.move_to_neutral()
    arm.gripper.close()
    arm.set_speed(0.25)

    while True:
        arm.align_to_base(z=True)

        ### OPEN PC
        grasp_tool(
            pose_dict["pc_open_tool"],
            "panda_tool_pc_open_tcp",
            tool_weights["panda_tool_pc_open"],
        )
        arm.move_to_cartesian(pose_dict["approx_over_pc"])
        pc_closed_z = find_pc_z()
        # pc_closed_x = find_pc_x() NOT WORKING RIGHT NOW, TOO CLOSE TO JOINT LIMITS
        arm.relative_move(2, 0.02)
        arm.align_to_base()
        pc_closed_y = find_pc_y()
        pc_open_pose = open_pc(pc_closed_y, pc_closed_z)
        place_tool(tool_pose=pose_dict["pc_open_tool"])

        ### REMOVE SCREEN FRAME
        grasp_tool(
            pose_dict["screen_frame_remover_tool"],
            "panda_tool_screen_frame_remover_tcp",
            tool_weights["panda_tool_frame_remover"],
        )
        arm.move_to_cartesian(pose_dict["approx_over_screen"])
        contact_with_screen_frame()
        remove_screen_frame()
        place_screen_frame_in_holder()
        arm.move_to_neutral()
        place_tool(pose_dict["screen_frame_remover_tool"])

        ### UNSCREW SCREWS
        ## Set this from the terminal or make rosnode   NE_T_EE: [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.088, 1.0]
        grasp_screw_tool(pose_dict["approx_over_screw_tool"], screw_direction=0)
        # arm.move_to_neutral()
        unscrew_screws_from_pc()
        place_screw_tool(pose_dict["approx_over_screw_tool"], screw_direction=0)

        ### REMOVE SCREEN
        grasp_tool(
            pose_dict["screen_remover_tool"],
            "panda_tool_screen_remover_tcp",
            tool_weights["panda_tool_screen_remover"],
        )
        dropoff_screen_pose = remove_screen()

        ### REATTACH SCREEN
        pickup_screen_from_holder(dropoff_screen_pose)
        replace_screen()
        arm.move_to_neutral()
        place_tool(pose_dict["screen_remover_tool"])
        arm.move_to_neutral()

        ### RESCREW SCREWS
        grasp_screw_tool(pose_dict["approx_over_screw_tool"], screw_direction=1)
        arm.move_group.set_end_effector_link("panda_tool_screw_tcp")
        # arm.move_to_neutral()
        rescrew_screws_to_pc()
        place_screw_tool(pose_dict["approx_over_screw_tool"], screw_direction=1)

        ### REPLACE SCREEN FRAME
        pickup_screen_frame_from_holder()
        replace_screen_frame()
        # arm.move_to_neutral()

        ### REATTACH SCREEN FRAME
        grasp_tool(
            pose_dict["pc_open_tool"],
            "panda_tool_pc_open_tcp",
            tool_weights["panda_tool_pc_open"],
        )
        arm.move_to_joint(pose_dict["above_screen_frame_reattach_startpoint"])
        reattach_screen_frame_NEW(force_z=8)
        arm.move_to_neutral()
        place_tool(pose_dict["pc_open_tool"])

        ### CLOSE PC
        grasp_tool(
            pose_dict["pc_open_tool"],
            "panda_tool_pc_open_tcp",
            tool_weights["panda_tool_pc_open"],
        )
        # arm.move_group.set_end_effector_link("panda_tool_pc_open_tcp")
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
