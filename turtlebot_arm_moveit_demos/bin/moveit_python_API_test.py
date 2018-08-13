#!/usr/bin/env python

# (various imports)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("moveit_python_API_test", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_arm = moveit_commander.MoveGroupCommander("arm")

REFERENCE_FRAME = "world"

# Move the robot arm
x = 0.11
y = 0.0
z = 0.028
roll = 0
pitch = np.pi/2
yaw = 0

pose = self.createPose(x, y, z, roll, pitch, yaw)
success = self.moveArmToPose(pose)

def createPose(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose
# createPose

def moveArmToPose(self, targetPose, execute=True):
    self.group_arm.clear_pose_targets()
    self.group_arm.set_pose_target(targetPose)
    planArm = self.group_arm.plan()
    if(execute):
        success = self.group_arm.go(wait=True)
        return success
# moveArmToPose
