import numpy as np
import rospy
from sensor_msgs.msg import JointState
from interbotix_xs_modules.arm import InterbotixManipulatorXS

# This script makes the end-effector perform pick, pour, and place tasks
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=vx300s'
# Then change to this directory and type 'python bartender.py'


class TestMotionRunner:
    MAX_EFFORT = [500.0, 1500.0, 1000.0, 500.0, 500.0, 500.0, 1000.0, 1000.0, 1000.0]

    def __init__(self):
        rospy.Subscriber("/vx300s/joint_states", JointState, self.callback_states)
        self.bot = InterbotixManipulatorXS("vx300s", "arm", "gripper")

    def callback_states(self, joint_states):
        if np.any(np.abs(joint_states.effort) > self.MAX_EFFORT):
            rospy.logwarn("Over effort!")
            rospy.logwarn(f"Current effort values: {joint_states.effort}")
            rospy.logwarn(f"Maximum effort values: {self.MAX_EFFORT}")
            rospy.signal_shutdown("Over effort")
            rospy.spin()

    def run(self):
        self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)
        self.bot.arm.set_single_joint_position("waist", np.pi / 2.0)
        self.bot.gripper.open()
        self.bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.14)
        self.bot.gripper.close()
        self.bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.14)
        self.bot.arm.set_single_joint_position("waist", -np.pi / 2.0)
        self.bot.arm.set_ee_cartesian_trajectory(pitch=1.5)
        self.bot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
        self.bot.arm.set_single_joint_position("waist", np.pi / 2.0)
        self.bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.14)
        self.bot.gripper.open()
        self.bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.14)
        self.bot.arm.go_to_home_pose()
        self.bot.arm.go_to_sleep_pose()


if __name__ == '__main__':
    runner = TestMotionRunner()
    runner.run()
