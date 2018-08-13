#!/usr/bin/env python

# BEGIN_SUB_TUTORIAL imports
##
# To use the python interface to move_group, import the moveit_commander
# module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
# END_SUB_TUTORIAL

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import tf

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_arm = moveit_commander.MoveGroupCommander("arm")
group_gripper = moveit_commander.MoveGroupCommander("gripper")

REFERENCE_FRAME = 'world'   # used to be: base_link, also possible: world (if defined in the urdf.xarco file)


def move_group_python_interface_tutorial():
    print "============ Starting python script"

    # Allow some leeway in position (meters) and orientation (radians)
    group_arm.set_goal_position_tolerance(0.01)
    group_arm.set_goal_orientation_tolerance(0.01)

    # Allow replanning to increase the odds of a solution
    group_arm.allow_replanning(True)

    # Set the right arm reference frame
    group_arm.set_pose_reference_frame(REFERENCE_FRAME)

    # Allow 5 seconds per planning attempt
    group_arm.set_planning_time(5)

    # Getting Basic Information
    # ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot
    print "============ Reference frame group_arm: %s" % group_arm.get_planning_frame()
    print "============ Reference frame group_gripper: %s" % group_gripper.get_planning_frame()

    # We can also print the name of the end-effector link for this group
    print "============ End effector group_arm: %s" % group_arm.get_end_effector_link()

    # We can get a list of all the groups in the robot
    print "============ Robot Groups:"
    print robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot.
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"

    import time
    start = time.time()

    moveArmToHomePosition()
    # rospy.sleep(5)
    print "============ current pose of group_arm:"
    pose_current = group_arm.get_current_pose().pose
    print(pose_current)
    # pose_current = geometry_msgs.msg.Pose() # uncomment this for autocompletion when writing code (Visual Studio Code: Ctrl+Shift+7)
    x = pose_current.position.x + 0.01
    y = pose_current.position.y + 0.01
    z = pose_current.position.z - 0.05

    quaternion = pose_current.orientation
    explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
    print "current orientation in euler format: roll=%.5f, pitch=%.5f, yaw=%.5f" % (roll, pitch, yaw)  # roll, pitch, yaw
    success = moveArmToPosition(x,y,z,roll,pitch,yaw)

    if(not success):
      print("======================== FAILED TO MOVE ARM ========================")


    print "============ current pose of group_arm:"
    pose_current = group_arm.get_current_pose().pose
    print(pose_current)
    quaternion = pose_current.orientation
    explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
    print "current orientation in euler format: roll=%.5f, pitch=%.5f, yaw=%.5f" % (roll, pitch, yaw)  # roll, pitch, yaw

    # print "============ moving arm to named position:"
    # # right_up, resting, forward, forward_left, forward_right, down, down_left, down_right
    # group_arm.set_named_target('forward_left')
    # planArm = group_arm.plan()
    # group_arm.execute(planArm)

    # moveGripper("grip_closed")

    rospy.Subscriber("/camera/depth/points", PointCloud2, callbackPointCloud)
    print "============ starting listening to PointCloud topic"

    # for i in range(1,10):
    #   moveArmToHomePosition()
    #   moveGripper("grip_open")
    #   moveArmToPosition(x,y,z,roll,pitch,yaw)

    # rospy.sleep(5)
    # scene.remove_world_object("box1")

    
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

    end = time.time()
    timeNeeded = end - start
    print("time needed for python script: %.5fs" % timeNeeded)

    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

    # END_TUTORIAL
    print "============ STOPPING"
# move_group_python_interface_tutorial


def moveGripper(namedPose):
    """ grip_open, grip_closed, grip_mid """
    print("============ moving gripper to pose: %s" % namedPose)
    group_gripper.set_named_target(namedPose)
    planGripper = group_gripper.plan()
    group_gripper.execute(planGripper)
# moveGripper


def moveArmToHomePosition():
    moveGripper("grip_open")
    arm_joint_values = group_arm.get_current_joint_values()
    if(not all(joint_values == 0.0 for joint_values in arm_joint_values)):
        print "============ going to home position"
        group_arm.clear_pose_targets()
        for i in range(len(arm_joint_values)):
            arm_joint_values[i] = 0.0
        group_arm.set_joint_value_target(arm_joint_values)
        plan0 = group_arm.plan()
        group_arm.execute(plan0)
        # rospy.sleep(2)
# moveArmToHomePosition


def moveArmToPosition(x, y, z, roll, pitch, yaw, execute=True):
    targetPose = geometry_msgs.msg.Pose()
    targetPose.position.x = x
    targetPose.position.y = y
    targetPose.position.z = z
    # when working with the orientation, the quaternion has to be normalized, so this doesn't work:
    # targetPose.orientation.x = 0.5
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    targetPose.orientation.x = quaternion[0]
    targetPose.orientation.y = quaternion[1]
    targetPose.orientation.z = quaternion[2]
    targetPose.orientation.w = quaternion[3]
    # group_arm.allow_replanning(true)
    group_arm.set_pose_target(targetPose)
    planArm = group_arm.plan()
    if(execute):
        # success = group_arm.execute(planArm)
        success = group_arm.go(wait=True)
        return success
# moveArmToPosition


def addObjectToWorld():
    rospy.sleep(2)
    # define a pose for the box (specified relative to frame_id)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = robot.get_planning_frame()
    box_pose.pose.orientation.w = 0.0
    box_pose.pose.position.x = 1.0
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = 1.5
    # Now, let's add the collision object into the world
    print "============ Add an object into the world"
    scene.remove_world_object("box1")
    scene.add_box("box1", box_pose, size=(0.4, 0.4, 0.1))
    # Sleep to allow MoveGroup to recieve and process the collision object message
    rospy.sleep(2)
# addObjectToWorld


def callbackPointCloud(data):
    rospy.loginfo(rospy.get_caller_id() + " Received Point Cloud: %s", data.is_dense)
    # pc = PointCloud2()
    # pc.is


if __name__ == '__main__':
    try:
        move_group_python_interface_tutorial()
    except rospy.ROSInterruptException as e:
        print(e)
