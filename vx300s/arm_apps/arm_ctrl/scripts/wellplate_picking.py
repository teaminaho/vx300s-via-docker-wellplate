#!/usr/bin/env python3

import sys
import yaml
import numpy as np
import rospy

from threading import Lock
from enum import IntEnum, unique

import actionlib
from arm_ctrl_msgs.msg import ArmControlAction, ArmControlGoal, ArmControlFeedback, ArmControlResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Time
from std_srvs.srv import Trigger, TriggerResponse
from interbotix_xs_modules.arm import InterbotixManipulatorXS


@unique
class ArmState(IntEnum):
    STOP_LATCHED = -1
    STOPPED = 0
    RUNNING = 1


@unique
class ArmMotion(IntEnum):
    HOME = 0
    DOOR_OPENNING = 1
    DOOR_CLOSING = 2
    WELLPLATE_PICKING_FROM_INCUBATOR = 3
    WELLPLATE_PICKING_FROM_STAGE = 4
    WELLPLATE_TO_STAGE_SEQUENCE = 11
    WELLPLATE_TO_INCUBATOR_SEQUENCE = 12
    HOME_AND_SLEEP = 100
    GRIPPER_OPEN = 101
    GRIPPER_CLOSE = 102


class RobotArm(object):
    OVER_EFFORT_COUNT_LIMIT = 3

    def __init__(self):
        self._lock = Lock()

        self._joint_states_watchdog = None
        self._over_effort_count = 0

        self._state = ArmState.STOPPED

        with open('/app/config/control/control_parameters.yaml', 'r') as f:
            config = yaml.safe_load(f)
        self._config = config

        self._over_effort_pub = rospy.Publisher(
            "/arm_ctrl/over_effort", Bool, queue_size=1, latch=True)

        bot = InterbotixManipulatorXS(
            "vx300s", "arm", "gripper", moving_time=2.0, accel_time=1.0)
        self._bot = bot

        self._health_pub = rospy.Publisher(
            "/arm_ctrl/health", Time, queue_size=1, latch=True)
        self._health_timer = rospy.Timer(
            rospy.Duration(0.2), self.callback_health_timer)

        rospy.Subscriber("/vx300s/joint_states",
                         JointState, self.callback_states)
        rospy.Service("/arm_ctrl/stop", Trigger, self.stop_handler)
        rospy.Service("/arm_ctrl/clear_latch", Trigger,
                      self.clear_latch_handler)

        self._as = actionlib.SimpleActionServer(
            '/arm_ctrl/run_arm', ArmControlAction, execute_cb=self.action_execute_cb, auto_start=False)
        self._as.register_preempt_callback(self.action_preempt_cb)
        self._as.start()

    def get_default_moving_time():
        return (2.5, 1.0)

    def is_movable(self):
        return self._state != ArmState.STOP_LATCHED

    def check_continue(self) -> bool:
        if self._as.is_preempt_requested():
            return False
        else:
            return True

    def locking(func):
        def try_lock(self, *args, **kwargs):
            if self._lock.acquire(blocking=False):
                print('Got lock')
                res = func(self, *args, **kwargs)
                self._lock.release()
                return res
            else:
                print('Unable to get lock')
                if self._as.is_active():
                    result = ArmControlResult(done=False)
                    self._as.set_aborted(result)
                return False
        return try_lock

    def mark_running(func):
        def dummy_gen():
            yield False

        def inner(self, *args, **kwargs):
            print('mark_running')
            # return False
            print(f"is movable: {self.is_movable()}")
            if not self.is_movable():
                return dummy_gen()
            print("MARK RUNNING")
            self._state = ArmState.RUNNING
            res = func(self, *args, **kwargs)
            return res
        return inner

    def check_proceed(func):
        def inner(self, *args, **kwargs):
            print('check_proceed')
            if not self.is_movable():
                return False
            if not self.check_continue():
                return False
            res = func(self, *args, **kwargs)
            return res
        return inner

    def check_proceed_for_task(func):
        def dummy_gen():
            yield False

        def inner(self, *args, **kwargs):
            print('check_proceed_for_task')
            if not self.is_movable():
                return dummy_gen()
            if not self.check_continue():
                return dummy_gen()
            res = func(self, *args, **kwargs)
            return res
        return inner

    def callback_health_timer(self, evt):
        self._health_pub.publish(rospy.get_rostime())

    def callback_states(self, joint_states):
        def _shutdown_with_error(msg: str):
            rospy.signal_shutdown(msg)
            rospy.spin()
            sys.exit(100)

        if len(joint_states.effort) == 0:
            self._over_effort_pub.publish(False)
            return
        # check if all base to wrist motors have no suspicious value
        if np.all(np.abs(np.array(joint_states.position)[:7] - (-3.14159)) < 1e-4):
            _shutdown_with_error('suspicious joint states')

        if self._joint_states_watchdog is not None:
            if (joint_states.header.stamp - self._joint_states_watchdog) >= rospy.Duration(0.1):
                _shutdown_with_error('joint state publication is unreliable')
            else:
                self._joint_states_watchdog = joint_states.header.stamp

        if np.any(np.abs(joint_states.effort) > self._config['max_effort']):
            self._over_effort_count += 1
        if self._over_effort_count >= RobotArm.OVER_EFFORT_COUNT_LIMIT:
            self._over_effort_pub.publish(True)
            if self._as.is_active():
                result = ArmControlResult(done=False)
                self._as.set_aborted(result)
            rospy.logwarn('Over effort!')
            rospy.logwarn(f"Current effort values: {joint_states.effort}")
            rospy.logwarn(
                f"Maximum effort values: {self._config['max_effort']}")
            _shutdown_with_error('over effort')
        else:
            self._over_effort_pub.publish(False)

    def action_preempt_cb(self):
        result = ArmControlResult(done=False)
        self._as.set_aborted(result)
        self._state = ArmState.STOP_LATCHED

    @locking
    def action_execute_cb(self, goal: ArmControlGoal):
        print('action_execute_cb ->')
        feedback = ArmControlFeedback()
        feedback.step = 1

        def run_task(task, set_succeeded=True) -> bool:
            print('run_task')
            print(task)
            for r in task():
                print(r)
                if r:
                    self._as.publish_feedback(feedback)
                else:
                    print('r = False')
                    result = ArmControlResult(done=False)
                    self._as.set_aborted(result)
                    self._state = ArmState.STOP_LATCHED
                    return False
                feedback.step += 1
                if not self.check_continue():
                    print('check_continue False')
                    result = ArmControlResult(done=False)
                    self._as.set_aborted(result)
                    self._state = ArmState.STOP_LATCHED
                    return False
            print('out of for r in task()')
            if set_succeeded:
                result = ArmControlResult(done=True)
                self._as.set_succeeded(result)
                self._state = ArmState.STOPPED
            return True

        if goal.motion_id == ArmMotion.DOOR_OPENNING:
            print('DOOR_OPENNING')
            run_task(self.open_incubator_door)
            print('<-action_execute_cb')
            return
        elif goal.motion_id == ArmMotion.DOOR_CLOSING:
            print('DOOR_CLOSING')
            run_task(self.close_incubator_door)
            print('<-action_execute_cb')
            return
        elif goal.motion_id == ArmMotion.WELLPLATE_PICKING_FROM_INCUBATOR:
            print('WELLPLATE_PICKING_FROM_INCUBATOR')
            run_task(self.take_out_wellplate)
            print('<-action_execute_cb')
            return
        elif goal.motion_id == ArmMotion.WELLPLATE_PICKING_FROM_STAGE:
            print('WELLPLATE_PICKING_FROM_STAGE')
            run_task(self.store_wellplate)
            print('<-action_execute_cb')
            return
        elif goal.motion_id == ArmMotion.WELLPLATE_TO_STAGE_SEQUENCE:
            print('WELLPLATE_TO_STAGE_SEQUENCE')
            if run_task(self.open_incubator_door, False) and \
               run_task(self.take_out_wellplate, False) and \
               run_task(self.close_incubator_door, True):
                pass
            print('<-action_execute_cb')
            return
        elif goal.motion_id == ArmMotion.WELLPLATE_TO_INCUBATOR_SEQUENCE:
            print('WELLPLATE_TO_INCUBATOR_SEQUENCE')
            if run_task(self.open_incubator_door, False) and \
               run_task(self.store_wellplate, False) and \
               run_task(self.close_incubator_door, True):
                pass
            print('<-action_execute_cb')
            return
        elif goal.motion_id == ArmMotion.HOME:
            print('HOME')
            run_task(self.home_task)
            print('<-action_execute_cb')
            return
        elif goal.motion_id == ArmMotion.HOME_AND_SLEEP:
            print('HOME_AND_SLEEP')
            run_task(self.home_and_sleep_task)
            print('<-action_execute_cb')
            return
        elif goal.motion_id == ArmMotion.GRIPPER_OPEN:
            print('GRIPPER_OPEN')
            run_task(self.gripper_open_task)
            print('<-action_execute_cb')
            return
        elif goal.motion_id == ArmMotion.GRIPPER_CLOSE:
            print('GRIPPER_OPEN')
            run_task(self.gripper_close_task)
            print('<-action_execute_cb')
            return
        else:
            result = ArmControlResult(done=False)
            self._as.set_aborted(result)
            print('<-action_execute_cb')
            return

    def stop_handler(self, req):
        if self._state in [ArmState.STOP_LATCHED, ArmState.STOPPED]:
            return TriggerResponse(success=True, message="already stopped")
        else:
            self._state = ArmState.STOP_LATCHED
            return TriggerResponse(success=True, message="going to stop")

    def clear_latch_handler(self, req):
        if self._state == ArmState.STOP_LATCHED:
            self._state = ArmState.STOPPED
            return TriggerResponse(success=True, message="now arm is ready to move")
        elif self._state == ArmState.STOPPED:
            return TriggerResponse(success=True, message="arm is already stopped")
        else:
            return TriggerResponse(success=False, message="arm is not latched stop")

    @mark_running
    @check_proceed_for_task
    def open_incubator_door(self):
        print('open_incubator_door')
        yield self.set_torque(True)
        yield self.home()
        yield self.open_gripper()

        # approaching to door lever
        yield self.set_joints([-4.0, 4.0, -22.5, -19.0, 4.5, 17.0], (2.5, 1.0))

        # grab lever
        yield self.close_gripper(0.9)

        # operate lever
        yield self.run_position_sequence([
            ([-4.0, -40.0, 25.0, -19.0, 4.5, 17.0], (2.5, 1.0)),
            ([-4.0, 4.0, -22.5, -19.0, 4.5, 17.0], (2.5, 1.0))])

        # release lever
        yield self.open_gripper()

        # leaving from lever
        yield self.run_position_sequence([
            ([-4.0, -8.0, -9.0, -19.0, 4.5, 17.0], (2.5, 1.0)),
            ([-13.0, -8.0, -9.0, -19.0, 4.5, 17.0], (2.5, 1.0)),
            ([-13.0, 15.0, 20.0, -19.0, -30.0, 17.0], (2.5, 1.0))])

        # open door
        yield self.run_position_sequence([
            ([-10.0, 0.0, 45.0, -19.0, -35.0, 17.0], (2.5, 1.0)),
            ([-5.0, -20.0, 65.0, -19.0, -35.0, 17.0], (2.5, 1.0)),
            ([10.0, -25.0, 75.0, -19.0, -45.0, 17.0], (2.5, 1.0)),
            ([40.0, -15.0, 70.0, -19.0, -50.0, 17.0], (2.5, 1.0))])

        yield self.home()

    @mark_running
    @check_proceed_for_task
    def close_incubator_door(self):
        yield self.set_torque(True)
        yield self.home()
        yield self.open_gripper()

        # close door
        yield self.run_position_sequence([
            ([80.0, -73.0, 88.0, 0.0, -14.0, 0.0], (2.5, 1.0)),
            ([80.0, 0.0, 60.0, 0.0, -65.0, 0.0], (2.5, 1.0)),
            ([70.0, 0.0, 60.0, 0.0, -65.0, 0.0], (2.5, 1.0)),
            ([60.0, -30.0, 83.0, 0.0, -57.0, 0.0], (2.5, 1.0)),
            ([10.0, -20.0, 80.0, 0.0, -65.0, 0.0], (2.5, 1.0)),
            ([-13.0, 0.0, 60.0, 0.0, -65.0, 0.0], (2.5, 1.0)),
            ([-12.0, 17.0, 35.0, 0.0, -55.0, 0.0], (2.5, 1.0))])

        yield self.home()

    @mark_running
    @check_proceed_for_task
    def take_out_wellplate(self):
        yield self.set_torque(True)
        yield self.home()
        yield self.open_gripper()

        # approach to wellplate
        yield self.run_position_sequence([
            ([21.5, 9.5, 13.5, 40.0, -33.0, -36.0], (2.5, 1.0)),
            ([21.0, 13.3, 7.5, 45.0, -30.5, -41.0], (2.5, 1.0))])

        # grab wellplate
        yield self.close_gripper(0.9)

        # take wellplate out of incubator
        yield self.run_position_sequence([
            ([22.5, 2.0, 20.0, 40.0, -32.0, -35.0], (2.5, 1.0)),
            ([30.0, -25.0, 47.0, 40.0, -34.0, -36.0], (2.5, 1.0))])

        yield self.upper_home((4.0, 1.5))

        # take wellplate in front of the stage
        yield self.set_joints([-89.0, -72.0, 85.0, 0.0, -16.0, 0.0], (3.5, 1.5))

        # move wellplate over the stage
        yield self.run_position_sequence([
            ([-89.0, -20.0, 76.0, 0.0, -60.0, 0.0], (2.5, 1.0)),
            ([-89.0, 2.7, 53.0, 0.0, -62.0, 0.0], (2.5, 1.0)),
            ([-89.0, 3.8, 54.5, 0.0, -63.0, 0.0], (2.5, 1.0))])

        # release wellplate
        yield self.open_gripper()

        # leaving from over the stage
        yield self.set_joints([-89.0, -70.0, 85.0, 0.0, -20.0, 0.0], (2.5, 1.0))

        yield self.home()

    @mark_running
    @check_proceed_for_task
    def store_wellplate(self):
        yield self.set_torque(True)
        yield self.home()
        yield self.open_gripper()

        # move in front of the stage
        yield self.run_position_sequence([
            ([-89.0, -20.0, 76.0, 0.0, -57.0, 0.0], (2.5, 1.0)),
            ([-88.5, 2.7, 53.0, 0.0, -57.0, 0.0], (2.5, 1.0))])

        # grab wellplate
        yield self.close_gripper(0.9)

        # pick wellplate up
        yield self.run_position_sequence([
            ([-88.5, 0.0, 53.0, 0.0, -57.0, 0.0], (2.5, 1.0)),
            ([-89.0, -20.0, 76.0, 0.0, -60.0, 0.0], (2.5, 1.0)),
            ([-89.0, -70.0, 85.0, 0.0, -20.0, 0.0], (2.5, 1.0))])

        # put wellplate into incubator
        yield self.run_position_sequence([
            ([40.0, -40.0, 60.0, 60.0, -43.0, -53.0], (4.0, 1.5)),
            ([45.0, -28.0, 52.0, 60.0, -52.0, -50.0], (3.5, 1.5)),
            ([21.5, 9.5, 13.5, 40.0, -33.0, -36.0], (2.5, 1.0)),
            ([21.0, 13.3, 8.0, 45.0, -30.5, -41.0], (2.5, 1.0))])

        # release wellplate
        yield self.open_gripper()

        # leave from incubator
        yield self.run_position_sequence([
            ([21.5, 9.5, 11.5, 40.0, -35.0, -36.0], (2.5, 1.0)),
            ([45.0, -28.0, 52.0, 60.0, -52.0, -50.0], (2.5, 1.0))])

        yield self.home()

    @mark_running
    @check_proceed_for_task
    def home_task(self):
        yield self.set_torque(True)
        yield self.home()

    @mark_running
    @check_proceed_for_task
    def home_and_sleep_task(self):
        yield self.set_torque(True)
        yield self.home()
        yield self.sleep()
        yield self.set_joints([0.0, -105.6, 92.0, 0.0, 42.0, 0.0], (2.5, 1.0))
        rospy.sleep(0.5)
        yield self.set_torque(False)

    @mark_running
    @check_proceed_for_task
    def gripper_open_task(self):
        yield self.set_torque(True)
        yield self.open_gripper()

    @mark_running
    @check_proceed_for_task
    def gripper_close_task(self):
        yield self.set_torque(True)
        yield self.close_gripper(0.9)

    def set_moving_time(self, time=get_default_moving_time()):
        moving_time, accel_time = time
        self._bot.arm.set_trajectory_time(
            moving_time=moving_time, accel_time=accel_time)
        rospy.sleep(0.2)

    @check_proceed
    def home(self, time=get_default_moving_time()):
        self.set_moving_time(time)
        _, b = self._bot.arm.set_ee_pose_components(x=0.2, y=0.0, z=0.2)
        return b

    @check_proceed
    def sleep(self, time=get_default_moving_time()):
        self.set_moving_time(time)
        return self._bot.arm.go_to_sleep_pose() is None

    @check_proceed
    def upper_home(self, time=(2.5, 1.0)):
        self.set_moving_time(time)
        _, b = self._bot.arm.set_ee_pose_components(x=0.17, y=0.0, z=0.27)
        return b

    @check_proceed
    def open_gripper(self):
        return self._bot.gripper.open() is None

    @check_proceed
    def close_gripper(self, pressure=0.5):
        self._bot.gripper.set_pressure(pressure)
        return self._bot.gripper.close() is None

    @check_proceed
    def set_torque(self, enable):
        return self._bot.dxl.robot_torque_enable(cmd_type='group', name='all', enable=enable) is None

    @check_proceed
    def run_position_sequence(self, sequence):
        if len(sequence) == 0 or not all([len(s) == 2 for s in sequence]) or \
                not all([len(s[0]) == 6 for s in sequence]) or not all([len(s[1]) == 2 for s in sequence]):
            rospy.logfatal(f'specified positions in sequence is invalid.')
            rospy.logfatal(sequence)
            raise ValueError
        for goal in sequence:
            positions, time = goal
            if not self.check_continue():
                return False
            if self.set_joints(positions, time=time) == False:
                return False
        return True

    @check_proceed
    def set_joints(self, degree_positions, time=get_default_moving_time()):
        moving_time, accel_time = time
        return self._bot.arm.set_joint_positions(np.radians(degree_positions), moving_time=moving_time, accel_time=accel_time)


if __name__ == '__main__':
    robot_arm = RobotArm()
    robot_arm.set_torque(True)
    rospy.spin()
