#!/usr/bin/env python

import rospy
from control_msgs.msg import GripperCommandActionGoal

J_POSES = {}
J_POSES["HANDLE_VIEW"] = [[-1.7536862514949982, -0.510532242025934, 1.506756357326197, 0.30385549510146276, -1.9141949703940737, -0.3170866120291951]]
J_POSES["CAN_VIEW"] = [[-1.6884036510616678, -1.1658008977110583, 1.1749062454780395, 0.2549310953478351, -1.3465794682989447, 0.832507885184296]]
J_POSES["DRIVE"] = [[-0.150818008324495, 0.21188589001666636, 2.5192843448199254, 1.3869593820111628, -1.0821904226558026, -1.396772597995887]]
J_POSES["STOW"] = [[-0.18278872457867834, 0.39905498195630723, 2.5428032530386333, 1.4640909187664735, -0.9262287850019284, -1.4092079623899023]]
J_POSES["INTO"] = [[-2.138385457725395, -0.2688980575912824, 1.4940630471528127, -0.1376280062205723, -2.1024596126725963, -0.21981584761200412], [-1.272334406650848, -0.1348152948675563, 1.6807822753421002, 0.060479608364980966, -1.2572919981326056, -1.8022910056228603], [-1.7653901549788653, -1.2609314427236855, 0.6401593788414426, -0.07517900709158479, -1.7293646539322485, -1.8899344727202312], [-1.6119464652617177, -1.002758320873392, 1.6022805594195104, -0.02710706099937387, -1.5650878337471636, -1.8899344727202312], [-1.6035382214789193, -0.5182801823152339, 1.6212485191425567, -0.022240144658530876, -1.570813358610594, -1.8899344727202312], [-1.5836087962164844, -1.3227684496229946, 0.6609285793795245, -0.14340545980539363, -1.7781820925356735, -2.1019377538830075]]
J_POSES["CLOSE"] = [[-0.03050516649293672, -0.4281143396600642, 1.900281094742744, 1.5554858961765563, -0.7385262307180253, -1.5440751909274948], [-0.1552088103606407, -0.6449595755383383, 1.2931183799018926, 0.05427146952291749, -1.458797517128221, -0.3484755154695029], [0.02373274644898614, -0.1922959009028406, 1.536245005909399, 0.059663560076150715, -1.2735670295667891, -0.1456458747866178], [0.5392787811768649, -0.43636118788523376, 1.1931187806110495, 0.05126913353532279, -1.0768270095992925, -0.057875718978236054]]
J_POSES["HANDOFF"] = [[-0.7514071605886776, 0.16530486400085984, 1.6618901758686713, 0.9074120691729844, -0.9163406648447804, -0.5786704404781242]]

LOCATIONS = {}
LOCATIONS["FRIDGE"] = [[12.043052369612, -2.04388162532], [-0.722147089553, 0.691739532664]]
LOCATIONS["FRIDGE2"] = [[12.043052369612, -2.04388162532], [-0.9997699, -0.021452]]
LOCATIONS["COUCH"] = [[10.8269882202, -8.50208568573], [-0.804190503373, 0.594371629777]]
LOCATIONS["OFFICE"] = [[0.45848762989, -2.8883368969], [-0.730518077094, 0.682893358468]]
LOCATIONS["THRONE"] = [[2.96381497383, -4.11378717422], [-0.793796384994, 0.608183606464]]

LOCATIONS["HOME"] = [[0, 0], [0,1]]

MAP_POSE = None

def cMove(pose_goal, group):
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

def jMoves(poses, group):
    for i in poses:
        group.go(i, wait=True)
        group.stop()

def getGoal(name):
    global LOCATIONS
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = '/map'
    goal.target_pose.pose.position.x = LOCATIONS[name][0][0]
    goal.target_pose.pose.position.y = LOCATIONS[name][0][1]
    goal.target_pose.pose.orientation.z = LOCATIONS[name][1][2]
    goal.target_pose.pose.orientation.w = LOCATIONS[name][1][3]
    return goal

def gripper(state, pub):
    msg = GripperCommandActionGoal()
    msg.max_effort = 100.0

    if state == "OPEN":
        msg.position = 0.0
    else:
        msg.position = 0.9

    pub.publish(msg)
    rospy.sleep(1.0)

def callback(data):
    global MAP_POSE
    if data.data in LOCATIONS.keys():
        MAP_POSE = data.data

def main():
    global MAP_POSE, J_POSES
    rospy.init_node('beer_sherpa_controller', anonymous=True)
    rospy.Subscriber('ifttt_data', String, callback)
    pub = rospy.Publisher('/arm/gen3_lite_2f_gripper_controller/gripper_cmd/goal', GripperCommandActionGoal, queue_size=10)

    listener = tf.TransformListener()

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    group.set_max_velocity_scaling_factor(1.0)
    group.set_max_acceleration_scaling_factor(0.5)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if MAP_POSE.any():
            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            client.wait_for_server()

            goal = getGoal("FRIDGE")
            client.send_goal(goal)
            client.wait_for_result()

            gripper("OPEN", pub)

            group.go(J_POSES["HANDLE_VIEW"], wait=True)
            group.stop()

            listener.waitForTransform(group.get_planning_frame(), '#1', rospy.Time(0), rospy.Duration(0.5))
            trans, rot = listener.lookupTransform(group.get_planning_frame(), '#1', rospy.Time(0))

            pose_goal = Pose()
            pose_goal.position.x = trans.x - 0.15
            pose_goal.position.y = trans.y
            pose_goal.position.z = trans.z + 0.05
            pose_goal.orientation.x = -0.5
            pose_goal.orientation.y = -0.5
            pose_goal.orientation.z = 0.5
            pose_goal.orientation.w = 0.5

            cMove(pose_goal, group)

            pose_goal.position.x = trans.x + 0.02
            pose_goal.position.y = trans.y
            pose_goal.position.z = trans.z + 0.05
            cMove(pose_goal, group)

            gripper("CLOSE", pub)

            pose_goal.position.x = trans.x - 0.4
            pose_goal.position.y = trans.y - 0.4
            pose_goal.position.z = trans.z + 0.05

            gripper("OPEN", pub)

            jMoves(J_POSES["INTO"], group)

            group.go(J_POSES["CAN_VIEW"], wait=True)
            group.stop()

            listener.waitForTransform(group.get_planning_frame(), '#2', rospy.Time(0), rospy.Duration(0.5))
            trans, rot = listener.lookupTransform(group.get_planning_frame(), '#2', rospy.Time(0))

            pose_goal = Pose()
            pose_goal.position.x = trans.x - 0.15
            pose_goal.position.y = trans.y + 0.02
            pose_goal.position.z = trans.z + 0.1
            pose_goal.orientation.x = 0
            pose_goal.orientation.y = 0.7070727
            pose_goal.orientation.z = 0
            pose_goal.orientation.w = 0.7071408
            cMove(pose_goal, group)

            pose_goal.position.x = trans.x - 0.06
            pose_goal.position.y = trans.y + 0.02
            pose_goal.position.z = trans.z

            gripper("CLOSE", pub)

            cMove(pose_goal, group)

            pose_goal.position.x = trans.x - 0.06
            pose_goal.position.y = trans.y + 0.02
            pose_goal.position.z = trans.z + 0.06

            cMove(pose_goal, group)

            pose_goal.position.x = trans.x - 0.15
            pose_goal.position.y = trans.y + 0.02
            pose_goal.position.z = trans.z + 0.15

            cMove(pose_goal, group)

            jMoves(J_POSES["STOW"], group)

            goal = getGoal("FRIDGE2")
            client.send_goal(goal)
            client.wait_for_result()

            jMoves(J_POSES["CLOSE"], group)

            jMoves(J_POSES["STOW"], group)

            goal = getGoal(MAP_POSE)
            client.send_goal(goal)
            client.wait_for_result()

            jMoves(J_POSES["HANDOFF"], group)
            gripper("OPEN", pub)
            jMoves(J_POSES["STOW"], group)

            goal = getGoal("HOME")
            client.send_goal(goal)
            client.wait_for_result()

            MAP_POSE = None

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
