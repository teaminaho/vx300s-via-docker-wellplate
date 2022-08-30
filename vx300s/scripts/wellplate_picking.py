import yaml
import numpy as np
import rospy

from threading import Lock
from enum import Enum, unique
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
from interbotix_xs_modules.arm import InterbotixManipulatorXS


@unique
class ArmState(Enum):
    STOP_LATCHED = -1
    STOPPED = 0
    GOING_TO_RUN = 1
    RUNNING = 2


@unique
class ArmMotion(Enum):
    HOME = 0
    DOOR_OPENNING = 1
    DOOR_CLOSING = 2
    WELLPLATE_PICKING_FROM_INCUBATOR = 3
    WELLPLATE_PICKING_FROM_STAGE = 4
    WELLPLATE_MOVING_ONTO_STAGE = 5
    WELLPLATE_MOVING_INTO_INCUBATOR = 6


class RobotArm(object):
    OVER_EFFORT_COUNT_LIMIT = 3

    def __init__(self):
        self._lock = Lock()
        with open('/app/config/control/control_parameters.yaml', 'r') as f:
            config = yaml.safe_load(f)

        rospy.Subscriber('/vx300s/joint_states',
                         JointState, self.callback_states)
        rospy.Service('/arm_ctrl/home_and_shutdown', Trigger,
                      self.home_and_shutdown_handler)
        rospy.Service('/arm_ctrl/home', Trigger, self.home_handler)
        rospy.Service('/arm_ctrl/stop', Trigger, self.stop_handler)
        rospy.Service('/arm_ctrl/clear_latch', Trigger,
                      self.clear_latch_handler)
        rospy.Service('/arm_ctrl/open_door', Trigger,
                      self.open_incubator_door_handler)
        rospy.Service('/arm_ctrl/close_door', Trigger,
                      self.close_incubator_door_handler)
        rospy.Service('/arm_ctrl/take_out_wellplate', Trigger,
                      self.take_out_wellplate_handler)
        rospy.Service('/arm_ctrl/store_wellplate', Trigger,
                      self.store_wellplate_handler)

        self._over_effort_pub = rospy.Publisher(
            '/arm_ctrl/over_effort', Bool, queue_size=1, latch=True)

        bot = InterbotixManipulatorXS('vx300s', 'arm', 'gripper')

        self._config = config
        self._bot = bot
        self._over_effort_count = 0
        self._state = ArmState.STOPPED

    def locking(func):
        def try_lock(self, *args, **kwargs):
            if self._lock.acquire(blocking=False):
                res = func(self, *args, **kwargs)
                self._lock.release()
                return res
            else:
                return TriggerResponse(success=False, message='arm is still moving')
        return try_lock

    def mark_running(func):
        def inner(self, *args, **kwargs):
            if not self.is_movable():
                return False
            self._state = ArmState.RUNNING
            res = func(self, *args, **kwargs)
            self._state = ArmState.STOPPED if res else ArmState.STOP_LATCHED
            return res
        return inner

    def callback_states(self, joint_states):
        if len(joint_states.effort) == 0:
            self._over_effort_pub.publish(False)
            return
        if np.any(np.abs(joint_states.effort) > self._config['max_effort']):
            self.over_effort_count += 1
        if self._over_effort_count >= RobotArm.OVER_EFFORT_COUNT_LIMIT:
            self._over_effort_pub.publish(True)
            rospy.logwarn('Over effort!')
            rospy.logwarn(f'Current effort values: {joint_states.effort}')
            rospy.logwarn(
                f'Maximum effort values: {self._config["max_effort"]}')
            rospy.signal_shutdown('Over effort')
            rospy.spin()
        else:
            self._over_effort_pub.publish(False)

    def stop_handler(self, req):
        if self._state in [ArmState.STOP_LATCHED, ArmState.STOPPED]:
            return TriggerResponse(success=True, message='already stopped')
        else:
            self._state = ArmState.STOP_LATCHED
            return TriggerResponse(success=True, message='going to stop')

    def clear_latch_handler(self, req):
        if self._state == ArmState.STOP_LATCHED:
            self._state = ArmState.STOPPED
            return TriggerResponse(success=True, message='now arm is ready to move')
        else:
            return TriggerResponse(success=False, message='arm is not latched stop')

    @locking
    def home_and_shutdown_handler(self, req):
        if self.is_movable():
            if not self.home():
                return TriggerResponse(success=False, message='failed to move to home position')
            if not self.sleep():
                return TriggerResponse(success=False, message='failed to move to sleep position')
            rospy.sleep(1)
            if not self.set_torque(False):
                return TriggerResponse(success=False, message='failed to set torque off')
            else:
                return TriggerResponse(success=True, message='arm is now shutdown')
        else:
            return TriggerResponse(success=False, message='arm is latched stop. release clear the latch before do any operation')

    @locking
    def home_handler(self, req):
        if self.is_movable():
            if self.home():
                return TriggerResponse(success=True, message='arm is on home position')
            else:
                return TriggerResponse(success=False, message='failed to move to home position')
        else:
            return TriggerResponse(success=False, message='arm is latched stop. release clear the latch before do any operation')

    @locking
    def open_incubator_door_handler(self, req):
        res = self.open_incubator_door()
        if res:
            return TriggerResponse(success=True, message='incubator door open is successfully done')
        else:
            return TriggerResponse(success=False, message='incubator door open is failed by set position or manually interrupted')

    @locking
    def close_incubator_door_handler(self, req):
        res = self.close_incubator_door()
        if res:
            return TriggerResponse(success=True, message='incubator door close is successfully done')
        else:
            return TriggerResponse(success=False, message='incubator door close is failed by set position or manually interrupted')

    @locking
    def take_out_wellplate_handler(self, req):
        res = self.take_out_wellplate()
        if res:
            return TriggerResponse(success=True, message='taking out wellplate is successfully done')
        else:
            return TriggerResponse(success=False, message='taking out wellplate is failed by set position or manually interrupted')

    @locking
    def store_wellplate_handler(self, req):
        res = self.store_wellplate()
        if res:
            return TriggerResponse(success=True, message='storing wellplate is successfully done')
        else:
            return TriggerResponse(success=False, message='storing wellplate is failed by set position or manually interrupted')

    @mark_running
    def open_incubator_door(self):
        if not self.set_torque(True):
            return False
        if not self.home():
            return False
        if not self.open_gripper():
            return False

        # approaching to door lever
        if not self.set_joints([-1.0, 4.0, -22.5, -19.0, 4.5, 17.0]):
            return False

        # grab lever
        if not self.close_gripper(0.9):
            return False

        # operate lever
        if not self.run_position_sequence([[-1.0, -40.0, 25.0, -19.0, 4.5, 17.0],
                                           [-1.0, 4.0, -22.5, -19.0, 4.5, 17.0]]):
            return False

        # release lever
        if not self.open_gripper():
            return False

        # leaving from lever
        if not self.run_position_sequence([[-0.0, -8.0, -9.0, -19.0, 4.5, 17.0],
                                           [-12.0, -8.0, -9.0, -19.0, 4.5, 17.0],
                                           [-12.0, 15.0, 20.0, -19.0, -30.0, 17.0]]):
            return False

        # open door
        if not self.run_position_sequence([[-10.0, 0.0, 45.0, -19.0, -35.0, 17.0],
                                           [-5.0, -20.0, 65.0, -19.0, -35.0, 17.0],
                                           [10.0, -25.0, 75.0, -19.0, -45.0, 17.0],
                                           [40.0, -15.0, 70.0, -19.0, -50.0, 17.0]]):
            return False

        if not self.home():
            return False

        return True

    @mark_running
    def close_incubator_door(self):
        if not self.set_torque(True):
            return False
        if not self.home():
            return False
        if not self.open_gripper():
            return False

        # close door
        if not self.run_position_sequence([[80.0, -73.0, 88.0, 0.0, -14.0, 0.0],
                                           [80.0, 0.0, 60.0, 0.0, -65.0, 0.0],
                                           [70.0, 0.0, 60.0, 0.0, -65.0, 0.0],
                                           [60.0, -20.0, 80.0, 0.0, -65.0, 0.0],
                                           [10.0, -20.0, 80.0, 0.0, -65.0, 0.0],
                                           [-10.0, 0.0, 60.0, 0.0, -65.0, 0.0],
                                           [-10.0, 18.0, 32.0, 0.0, -55.0, 0.0],
                                           [-10.0, 0.0, 60.0, 0.0, -65.0, 0.0]]):
            return False

        if not self.home():
            return False

        return True

    @mark_running
    def take_out_wellplate(self):
        if not self.set_torque(True):
            return False
        if not self.home():
            return False
        if not self.open_gripper():
            return False

        # approach to wellplate
        if not self.run_position_sequence([[24.8, 9.0, 12.5, 40.0, -34.5, -36.0],
                                           [22.0, 15.0, 5.0, 40.0, -34.5, -36.0]]):
            return False

        # grab wellplate
        if not self.close_gripper(0.9):
            return False

        # take wellplate out of incubator
        if not self.run_position_sequence([[23.0, 9.0, 10.0, 40.0, -32.0, -36.0],
                                           [30.0, -25.0, 47.0, 40.0, -34.0, -36.0]]):
            return False

        if not self.upper_home():
            return False

        # take wellplate in front of the stage
        if not self.set_joints([-89.0, -70.0, 85.0, 0.0, -20.0, 0.0]):
            return False

        # move wellplate over the stage
        if not self.run_position_sequence([[-89.0, -18.0, 76.0, 0.0, -66.0, 0.0],
                                           [-89.0, -10.0, 67.0, 0.0, -65.0, 0.0],
                                           [-89.0, -8.0, 67.0, 0.0, -65.0, 0.0]]):
            return False

        # release wellplate
        if not self.close_gripper(0.9):
            return False

        # leaving from over the stage
        if not self.set_joints([-89.0, -70.0, 85.0, 0.0, -20.0, 0.0]):
            return False

        if not self.home():
            return False

        return True

    @mark_running
    def store_wellplate(self):
        if not self.set_torque(True):
            return False
        if not self.home():
            return False
        if not self.open_gripper():
            return False

        # move in front of the stage
        if not self.run_position_sequence([[-89.0, -70.0, 85.0, 0.0, -20.0, 0.0],
                                           [-89.0, -8.0, 67.0, 0.0, -65.0, 0.0]]):
            return False

        # grab wellplate
        if not self.close_gripper(0.9):
            return False

        # pick wellplate up
        if not self.set_joints([-89.0, -70.0, 85.0, 0.0, -20.0, 0.0]):
            return False

        # put wellplate into incubator
        if not self.run_position_sequence([[30.0, -30.0, 52.0, 40.0, -34.0, -36.0],
                                           [23.0, 9.0, 10.0, 40.0, -32.0, -36.0],
                                           [22.0, 15.0, 5.0, 40.0, -34.5, -36.0]]):
            return False

        # release wellplate
        if not self.open_gripper():
            return False

        # leave from incubator
        if not self.run_position_sequence([[23.0, 9.0, 10.0, 40.0, -32.0, -36.0],
                                           [30.0, -25.0, 47.0, 40.0, -34.0, -36.0]]):
            return False

        if not self.home():
            return False

        return True

    def set_default_moving_time(self):
        self._bot.arm.set_trajectory_time(moving_time=2, accel_time=1)

    def home(self):
        if not self.is_movable():
            return False
        self.set_default_moving_time()
        return self._bot.arm.set_ee_pose_components(x=0.2, y=0.0, z=0.2)

    def sleep(self):
        if not self.is_movable():
            return False
        self.set_default_moving_time()
        return self._bot.arm.go_to_sleep_pose() is None

    def upper_home(self):
        if not self.is_movable():
            return False
        return self._bot.arm.set_ee_pose_components(x=0.15, y=0.0, z=0.25)

    def open_gripper(self):
        if not self.is_movable():
            return False
        return self._bot.gripper.open() is None

    def close_gripper(self, pressure=0.5):
        if not self.is_movable():
            return False
        self._bot.gripper.set_pressure(pressure)
        return self._bot.gripper.close() is None

    def set_torque(self, enable):
        if not self.is_movable():
            return False
        return self._bot.dxl.robot_torque_enable(cmd_type='group', name='all', enable=enable) is None

    def run_position_sequence(self, positions_sequence):
        if not self.is_movable():
            return False
        if len(positions_sequence) == 0 or not all([len(s) == 6 for s in positions_sequence]):
            rospy.logfatal(f'specified positions in sequence is invalid.')
            rospy.logfatal(positions_sequence)
            raise ValueError
        for positions in positions_sequence:
            if self.set_joints(positions) == False:
                return False
        return True

    def set_joints(self, degree_positions):
        if not self.is_movable():
            return False
        return self._bot.arm.set_joint_positions(np.radians(degree_positions))

    def is_movable(self):
        return self._state != ArmState.STOP_LATCHED


if __name__ == '__main__':
    robot_arm = RobotArm()
    robot_arm.set_torque(True)
    rospy.spin()
