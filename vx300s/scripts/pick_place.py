import numpy as np
import rospy
from sensor_msgs.msg import JointState
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface

# This script uses a color/depth camera to get the arm to find objects and pick them up.
# For this demo, the arm is placed to the left of the camera facing outward. When the
# end-effector is located at x=0, y=-0.3, z=0.2 w.r.t. the 'vx300s/base_link' frame, the AR
# tag should be clearly visible to the camera. A small basket should also be placed in front of the arm.
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_perception xsarm_perception.launch robot_model:=vx300s'
# Then change to this directory and type 'python pick_place.py'


class PickPlaceRunner:
    MAX_EFFORT = [1200.0, 1500.0, 1000.0, 500.0, 500.0, 500.0, 1000.0, 1000.0, 1000.0]

    def __init__(self):
        rospy.Subscriber("/vx300s/joint_states", JointState, self.callback_states)

        # Initialize the arm module along with the pointcloud and armtag modules
        self.bot = InterbotixManipulatorXS("vx300s", moving_time=1.5, accel_time=0.75)
        self.pcl = InterbotixPointCloudInterface()

    def callback_states(self, joint_states):
        if np.any(np.abs(joint_states.effort) > self.MAX_EFFORT):
            rospy.logwarn("Over effort!")
            rospy.logwarn(f"Current effort values: {joint_states.effort}")
            rospy.logwarn(f"Maximum effort values: {self.MAX_EFFORT}")
            rospy.signal_shutdown("Over effort")
            rospy.spin()

    def run(self):
        while True:
            # set initial arm and gripper pose
            self.bot.arm.set_ee_pose_components(x=0.16, z=0.2, pitch=0.5)
            self.bot.gripper.open()

            # get the cluster positions
            # sort them from max to min 'x' position w.r.t. the 'vx300s/base_link' frame
            success, clusters = self.pcl.get_cluster_positions(ref_frame="vx300s/base_link", sort_axis="x", reverse=True)
            print(f"success: {success}, N: {len(clusters)}")

            # pick up all the objects and drop them in a virtual basket in front of the robot
            for i, cluster in enumerate(clusters):
                print(f"Target: {i}")
                x, y, z = cluster["position"]

                # picking
                self.bot.arm.set_ee_pose_components(x=x, y=y, z=z + 0.06, pitch=0.5)
                self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)
                self.bot.gripper.close()

                # placing
                self.bot.arm.set_ee_pose_components(x=x, y=y, z=z + 0.06, pitch=0.5)
                self.bot.arm.set_ee_pose_components(x=0.1, y=0.25, z=z + 0.06, pitch=0.5)
                self.bot.arm.set_ee_pose_components(x=0.1, y=0.25, z=z, pitch=0.5)
                self.bot.gripper.open()
                self.bot.arm.set_ee_pose_components(x=0.1, y=0.25, z=z + 0.06, pitch=0.5)

            # back to the initial arm and gripper pose
            self.bot.arm.set_ee_pose_components(x=0.16, z=0.2, pitch=0.5)

            key = input("Enter 'q' key to exit, enter other keys to continue.\n")
            if key == 'q':
                self.bot.arm.go_to_sleep_pose()
                break


if __name__ == '__main__':
    runner = PickPlaceRunner()
    runner.run()
