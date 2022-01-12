import time
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface

# This script uses a color/depth camera to get the arm to find objects and pick them up.
# For this demo, the arm is placed to the left of the camera facing outward. When the
# end-effector is located at x=0, y=-0.3, z=0.2 w.r.t. the 'vx300s/base_link' frame, the AR
# tag should be clearly visible to the camera. A small basket should also be placed in front of the arm.
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_perception xsarm_perception.launch robot_model:=vx300s'
# Then change to this directory and type 'python pick_place.py'


class PickPlaceRunner:
    MAX_EFFORT = [500.0, 1000.0, 1000.0, 500.0, 500.0, 500.0, 1000.0, 1000.0, 1000.0]

    def __init__(self, use_calibration=False):
        rospy.Subscriber("/vx300s/joint_states", JointState, self.callback_states)

        # Initialize the arm module along with the pointcloud and armtag modules
        self.bot = InterbotixManipulatorXS("vx300s", moving_time=1.5, accel_time=0.75)
        self.pcl = InterbotixPointCloudInterface()
        self.use_calibration = use_calibration

    def callback_states(self, joint_states):
        if np.any(np.abs(joint_states.effort) > self.MAX_EFFORT):
            rospy.logwarn("Over effort!")
            rospy.logwarn(f"Current effort values: {joint_states.effort}")
            rospy.logwarn(f"Maximum effort values: {self.MAX_EFFORT}")
            rospy.signal_shutdown("Over effort")
            rospy.spin()

    def run(self):
        # set initial arm and gripper pose
        self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)
        self.bot.gripper.open()

        # get the ArmTag pose
        if self.use_calibration:
            armtag = InterbotixArmTagInterface()
            self.bot.arm.set_ee_pose_components(y=-0.3, z=0.2)
            time.sleep(0.5)
            armtag.find_ref_to_arm_base_transform()
            self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)

        # get the cluster positions
        # sort them from max to min 'x' position w.r.t. the 'vx300s/base_link' frame
        success, clusters = self.pcl.get_cluster_positions(ref_frame="vx300s/base_link", sort_axis="x", reverse=True)
        print(f"success: {success}, N: {len(clusters)}")

        # pick up all the objects and drop them in a virtual basket in front of the robot
        for i, cluster in enumerate(clusters):
            print(f"Target: {i}")
            x, y, z = cluster["position"]
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=z + 0.05, pitch=0.5)
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)
            self.bot.gripper.close()
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=z + 0.05, pitch=0.5)
            self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)
            self.bot.gripper.open()
        self.bot.arm.go_to_sleep_pose()


if __name__ == '__main__':
    runner = PickPlaceRunner()
    runner.run()
