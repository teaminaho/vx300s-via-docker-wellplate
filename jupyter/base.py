import yaml
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from interbotix_xs_modules.arm import InterbotixManipulatorXS

bot = None
config = None
over_effort_count = 0
OVER_EFFORT_COUNT_LIMIT = 3

def callback_states(joint_states):
    global over_effort_count

    if np.any(np.abs(joint_states.effort) > config['max_effort']):
        over_effort_count += 1
    if over_effort_count >= OVER_EFFORT_COUNT_LIMIT:
        rospy.logwarn("Over effort!")
        rospy.logwarn(f"Current effort values: {joint_states.effort}")
        rospy.logwarn(f"Maximum effort values: {config['max_effort']}")
        rospy.signal_shutdown("Over effort")
        rospy.spin()

        
def init():
    global config
    global bot
    with open('/app/config/control/control_parameters.yaml', 'r') as f:
        config = yaml.safe_load(f)
    rospy.Subscriber("/vx300s/joint_states", JointState, callback_states)
    bot = InterbotixManipulatorXS("vx300s", "arm", "gripper")
    return config, bot

def set_default_moving_time():
    bot.arm.set_trajectory_time(moving_time=2, accel_time=1)

def home():
    set_default_moving_time()
    bot.arm.set_ee_pose_components(x=0.2, y=0.0, z=0.2)

def upper_home():
    bot.arm.set_ee_pose_components(x=0.15, y=0.0, z=0.25)