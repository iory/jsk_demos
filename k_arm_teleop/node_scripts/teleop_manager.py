#!/usr/bin/env python3
"""Leader -> K_ARM teleoperation, driven from the rviz TeleopPanel.

Target
    ``virtual`` sends to the ghost robot (fake controllers under /virtual),
    ``real`` to the robot's FollowJointTrajectory controllers.
Start
    First moves the follower slowly to the leader's pose (``approach``),
    then streams the leader's pose (``follow``). If the leader stops
    publishing, streaming stops and the next start approaches again.
Go zero
    Slowly moves the target robot to all-zero joint angles.
Recording
    Runs ``rosbag record`` into the chosen directory and reports how many
    bags it holds and how big they are.
"""

import datetime
import glob
import os
import shutil
import signal
import subprocess
import threading

import actionlib
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from k_arm_teleop.mapping import build_arm_maps
from k_arm_teleop.mapping import follower_to_leader
from k_arm_teleop.mapping import clamp
from k_arm_teleop.kinematics import rpy_to_matrix
from k_arm_teleop.kinematics import UrdfKinematics
from k_arm_teleop.kinematics import WristMap
from k_arm_teleop.mapping import leader_to_follower
from k_arm_teleop.mapping import leader_zero_pose
from k_arm_teleop.mapping import wrist_follower_joints
from k_arm_teleop.msg import LeaderServoStatus
from k_arm_teleop.msg import TeleopStatus
from k_arm_teleop.srv import MoveLeader
from k_arm_teleop.srv import SetArmEnabled
from k_arm_teleop.srv import SetArmEnabledResponse
from k_arm_teleop.srv import SetString
from k_arm_teleop.srv import SetStringResponse
from k_arm_teleop.urdf_utils import parse_joints


TARGETS = ('virtual', 'real')
# [rad] the leader wrist solution must reproduce the hand rotation this well
WRIST_SOLVE_TOLERANCE = 0.05
IDLE = 'idle'
APPROACHING = 'approaching'
FOLLOWING = 'following'
MOVING_TO_ZERO = 'moving_to_zero'
LEADER_LOST = 'leader_lost'
ERROR = 'error'
ACTIVE_PHASES = (APPROACHING, FOLLOWING, MOVING_TO_ZERO, LEADER_LOST)


class JointStateCache(object):
    """Latest positions from a JointState topic, merged across messages."""

    def __init__(self, topic):
        self.lock = threading.Lock()
        self.positions = {}
        self.stamps = []
        self.last_time = None
        rospy.Subscriber(topic, JointState, self._callback, queue_size=10)

    def _callback(self, msg):
        now = rospy.get_time()
        with self.lock:
            for name, position in zip(msg.name, msg.position):
                self.positions[name] = position
            self.last_time = now
            self.stamps.append(now)
            if len(self.stamps) > 50:
                del self.stamps[0]

    def snapshot(self):
        with self.lock:
            return dict(self.positions), self.last_time

    def rate(self):
        with self.lock:
            if len(self.stamps) < 2:
                return 0.0
            span = self.stamps[-1] - self.stamps[0]
            return (len(self.stamps) - 1) / span if span > 0 else 0.0


class Recorder(object):
    """A ``rosbag record`` child process plus statistics of its directory."""

    def __init__(self, directory, prefix, topics):
        self.directory = os.path.expanduser(directory)
        self.prefix = prefix
        self.topics = topics
        self.process = None
        self.bag_path = None
        self.started = None

    def set_directory(self, directory):
        if self.process is not None:
            raise RuntimeError('stop recording before changing the directory')
        directory = os.path.abspath(os.path.expanduser(directory))
        if not os.path.isdir(directory):
            os.makedirs(directory)
        self.directory = directory

    def start(self):
        if self.process is not None:
            raise RuntimeError('already recording')
        if not os.path.isdir(self.directory):
            os.makedirs(self.directory)
        stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
        self.bag_path = os.path.join(self.directory, '{}_{}.bag'.format(self.prefix, stamp))
        command = ['rosbag', 'record', '-O', self.bag_path,
                   '__name:=k_arm_teleop_recorder'] + list(self.topics)
        # own process group, so SIGINT reaches rosbag's children too
        self.process = subprocess.Popen(command, preexec_fn=os.setsid)
        self.started = rospy.get_time()

    def stop(self):
        if self.process is None:
            raise RuntimeError('not recording')
        os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
        try:
            self.process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            self.process.wait()
        path = self.bag_path
        self.process = None
        self.started = None
        return path

    def poll(self):
        """Return an error message if the child died on its own, else None."""
        if self.process is None or self.process.poll() is None:
            return None
        code = self.process.returncode
        self.process = None
        self.started = None
        return 'rosbag record exited with code {}'.format(code)

    def current_size(self):
        for path in (self.bag_path + '.active', self.bag_path):
            if self.bag_path and os.path.exists(path):
                return os.path.getsize(path)
        return 0

    def directory_stats(self):
        if not os.path.isdir(self.directory):
            return 0, 0, 0
        bags = glob.glob(os.path.join(self.directory, '*.bag')) + \
            glob.glob(os.path.join(self.directory, '*.bag.active'))
        total = sum(os.path.getsize(b) for b in bags if os.path.exists(b))
        finished = len([b for b in bags if b.endswith('.bag')])
        return finished, total, shutil.disk_usage(self.directory).free


class TeleopManager(object):

    def __init__(self):
        self.lock = threading.RLock()
        self.arms = build_arm_maps(rospy.get_param('~arms'))
        self.targets = rospy.get_param('~targets')
        self.real_is_simulated = rospy.get_param('~real_is_simulated', False)
        self.leader_source = rospy.get_param('~leader_source', 'feetech')
        # the K_ARM model this node maps onto (teleop.launch sets it from the URDF file)
        self.follower_description = rospy.get_param('~follower_description')
        self.limits = parse_joints(self.follower_description)
        self.margin = rospy.get_param('~follower_limit_margin', 0.02)
        self.leader_timeout = rospy.get_param('~leader_timeout', 0.3)
        self.approach = rospy.get_param('~approach')
        self.follow = rospy.get_param('~follow')
        self.zero_pose = rospy.get_param('~zero_pose')
        self.calibrate_service = rospy.get_param('~leader_calibrate_service')
        self.leader_move = rospy.get_param('~leader_move')
        self.leader_motion = ''
        rospy.Subscriber(self.leader_move['status_topic'], LeaderServoStatus,
                         self._leader_status_callback, queue_size=1)
        self.leader_description = rospy.get_param('~leader_description',
                                                  '/teleop_leader/robot_description')

        missing = [j for arm in self.arms.values() for j in self._arm_joints(arm)
                   if j not in self.limits]
        if missing:
            raise RuntimeError('follower joints {} are not in ~follower_description'.format(missing))
        self._build_wrist_maps(rospy.get_param('~leader_base'))

        self.leader = JointStateCache(rospy.get_param('~leader_joint_states'))
        self.leader_calibrated = self.leader_source != 'feetech'
        rospy.Subscriber(rospy.get_param('~leader_calibrated_topic'), Bool,
                         self._calibrated_callback, queue_size=1)
        self.followers = {name: JointStateCache(cfg['joint_states'])
                          for name, cfg in self.targets.items()}
        self.clients = {}
        self.command_pubs = {}

        recording = rospy.get_param('~recording')
        self.recorder = Recorder(recording['directory'], recording['prefix'],
                                 recording['topics'])

        self.target = 'virtual'
        self.phase = IDLE
        self.message = 'ready'
        self.pending_goals = []
        self.approach_attempts = 0
        self.last_command = {}
        self.max_tracking_error = 0.0
        self.clamped = []
        self.speed_limited = []

        self.command_pub = rospy.Publisher('~command', JointState, queue_size=10)
        self.status_pub = rospy.Publisher('~status', TeleopStatus, queue_size=1)
        rospy.Service('~set_target', SetString, self._set_target)
        rospy.Service('~set_arm_enabled', SetArmEnabled, self._set_arm_enabled)
        rospy.Service('~start', Trigger, self._start)
        rospy.Service('~stop', Trigger, self._stop)
        rospy.Service('~go_zero', Trigger, self._go_zero)
        rospy.Service('~calibrate_leader_zero', Trigger, self._calibrate_leader)
        rospy.Service('~leader_to_init', Trigger, self._leader_to_init)
        rospy.Service('~leader_to_robot', Trigger, self._leader_to_robot)
        rospy.Service('~leader_release', Trigger, self._leader_release)
        rospy.Service('~start_recording', Trigger, self._start_recording)
        rospy.Service('~stop_recording', Trigger, self._stop_recording)
        rospy.Service('~set_record_directory', SetString, self._set_record_directory)
        rospy.on_shutdown(self._shutdown)

    # -- helpers ------------------------------------------------------------

    def _calibrated_callback(self, msg):
        if self.leader_source == 'feetech':
            self.leader_calibrated = msg.data

    @staticmethod
    def _arm_joints(arm):
        return [j.follower for j in arm['joints']] + wrist_follower_joints(arm)

    def _build_wrist_maps(self, leader_base):
        """Wrist maps need both URDFs and the leader base rotation (teleop.yaml leader_base)."""
        leader = UrdfKinematics(rospy.get_param(self.leader_description))
        follower = UrdfKinematics(self.follower_description)
        base_rotation = rpy_to_matrix(*leader_base['rpy'])
        zero_pose = leader_zero_pose(rospy.get_param('~arms'))
        self.leader_zero = zero_pose
        for arm in self.arms.values():
            if not arm['wrist']:
                continue
            leader_joints = [j.leader for j in arm['joints']] + list(arm['wrist']['leader'])
            follower_joints = self._arm_joints(arm)
            arm['wrist_map'] = WristMap(leader, follower, leader_joints, follower_joints,
                                        zero_pose, base_rotation)

    def _grippers_held_back(self):
        """Grippers of enabled arms that the current target does not get."""
        if self.target != 'real' or self.real_is_simulated:
            return []
        return [arm['gripper']['follower'] for arm in self.arms.values()
                if arm['enabled'] and arm['gripper'] is not None
                and not arm['gripper'].get('send_to_real', False)]

    def _controller_entries(self, target):
        """[(controller, joints, kind)] that are commanded on ``target``.

        kind is 'arm' or 'gripper'. A gripper with send_to_real false is left
        out on the real robot (not on its simulated stand-in).
        """
        entries = []
        for arm in self.arms.values():
            if not arm['enabled']:
                continue
            entries.append((arm['controller'], self._arm_joints(arm), 'arm'))
            gripper = arm['gripper']
            # send_to_real only guards the hardware: the simulated stand-in gets
            # the gripper like the virtual robot does
            if gripper is not None and (target != 'real' or self.real_is_simulated
                                        or gripper.get('send_to_real', False)):
                entries.append((gripper['controller'], [gripper['follower']], 'gripper'))
        return entries

    def _client(self, target, controller):
        key = (target, controller)
        if key not in self.clients:
            namespace = self.targets[target]['namespace'].rstrip('/')
            name = '{}/{}/{}'.format(namespace, controller,
                                     self.targets[target].get('action', 'follow_joint_trajectory'))
            self.clients[key] = actionlib.SimpleActionClient(name, FollowJointTrajectoryAction)
            self.command_pubs[key] = rospy.Publisher(
                '{}/{}/command'.format(namespace, controller), JointTrajectory, queue_size=1)
        return self.clients[key]

    def _check_servers(self, target):
        missing = []
        for controller, _, _ in self._controller_entries(target):
            if not self._client(target, controller).wait_for_server(rospy.Duration(1.0)):
                missing.append(controller)
        if missing:
            raise RuntimeError('no action server for {} on target "{}"'.format(
                ', '.join(missing), target))

    def _leader_alive(self):
        _, last = self.leader.snapshot()
        return last is not None and rospy.get_time() - last < self.leader_timeout

    def _leader_targets(self):
        """Follower targets from the leader, for the joints commanded on the current target."""
        positions, _ = self.leader.snapshot()
        mapped = {}
        clamped = []
        for arm in self.arms.values():
            if arm['enabled']:
                result = leader_to_follower(arm, positions, self.limits, self.margin,
                                            previous=self.last_command)
                mapped.update(result['arm'])
                mapped.update(result['gripper'])
                clamped.extend(result['clamped'])
        commanded = [j for _, joints, _ in self._controller_entries(self.target) for j in joints]
        self.clamped = [j for j in clamped if j in commanded]
        return {j: mapped[j] for j in commanded}

    def _follower_positions(self, target):
        positions, last = self.followers[target].snapshot()
        if last is None:
            raise RuntimeError('no joint states from target "{}" ({})'.format(
                target, self.targets[target]['joint_states']))
        return positions

    def _send_goals(self, target, goal_positions, duration):
        """Send one-point trajectories reaching ``goal_positions`` after ``duration``."""
        self.pending_goals = []
        for controller, joints, _ in self._controller_entries(target):
            if not all(j in goal_positions for j in joints):
                continue
            trajectory = JointTrajectory()
            trajectory.joint_names = joints
            trajectory.points = [JointTrajectoryPoint(
                positions=[goal_positions[j] for j in joints],
                velocities=[0.0] * len(joints),
                time_from_start=rospy.Duration(duration))]
            client = self._client(target, controller)
            client.send_goal(FollowJointTrajectoryGoal(trajectory=trajectory))
            self.pending_goals.append(client)

    def _goals_state(self):
        """'running', 'done' or an error message."""
        states = [client.get_state() for client in self.pending_goals]
        if any(s in (GoalStatus.PENDING, GoalStatus.ACTIVE) for s in states):
            return 'running'
        failed = [s for s in states if s != GoalStatus.SUCCEEDED]
        if failed:
            return 'trajectory goal ended with status {}'.format(failed)
        return 'done'

    def _cancel_goals(self):
        for client in self.pending_goals:
            if client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                client.cancel_goal()
        self.pending_goals = []

    def _slow_move(self, target, goal_positions, velocity, min_duration):
        current = self._follower_positions(target)
        missing = [j for j in goal_positions if j not in current]
        if missing:
            raise RuntimeError('target "{}" does not report {}'.format(target, missing))
        distance = max([abs(goal_positions[j] - current[j]) for j in goal_positions] or [0.0])
        duration = max(float(min_duration), distance / float(velocity))
        self._send_goals(target, goal_positions, duration)
        self.last_command = dict(goal_positions)
        return distance, duration

    def _set_phase(self, phase, message):
        if phase != self.phase or message != self.message:
            log = rospy.logerr if phase == ERROR else rospy.loginfo
            log('[%s] %s: %s', self.target, phase, message)
        self.phase = phase
        self.message = message

    # -- services -------------------------------------------------------------

    def _set_target(self, request):
        with self.lock:
            if request.data not in TARGETS:
                return SetStringResponse(False, 'target must be one of {}'.format(TARGETS))
            if self.phase in ACTIVE_PHASES:
                return SetStringResponse(False, 'stop before changing the target')
            self.target = request.data
            self._set_phase(IDLE, 'target set to {}'.format(request.data))
            return SetStringResponse(True, self.message)

    def _set_arm_enabled(self, request):
        with self.lock:
            if request.arm not in self.arms:
                return SetArmEnabledResponse(False, 'unknown arm {}'.format(request.arm))
            if self.phase in ACTIVE_PHASES:
                return SetArmEnabledResponse(False, 'stop before changing the arms')
            self.arms[request.arm]['enabled'] = request.enabled
            return SetArmEnabledResponse(True, '{} {}'.format(
                request.arm, 'enabled' if request.enabled else 'disabled'))

    def _start(self, _request):
        with self.lock:
            if self.phase in ACTIVE_PHASES:
                return TriggerResponse(False, 'already {}'.format(self.phase))
            if not self._controller_entries(self.target):
                return TriggerResponse(False, 'no arm is enabled')
            if not self._leader_alive():
                return TriggerResponse(False, 'no data from the leader arm')
            if self.target == 'real' and not self.leader_calibrated:
                return TriggerResponse(False, 'leader is not calibrated; refusing to drive the '
                                              'real robot')
            if self.leader_motion == 'moving':
                return TriggerResponse(False, 'the leader is still moving to its pose')
            if self.leader_motion == 'holding':
                # hold the leader before pressing Start: its torque goes off here
                self._call_leader_trigger(self.leader_move['release_service'])
            for arm in self.arms.values():
                if arm['wrist_map'] is not None:
                    arm['wrist_map'].reset()
            self.last_command = {}
            try:
                self._check_servers(self.target)
                distance, duration = self._slow_move(
                    self.target, self._leader_targets(),
                    self.approach['velocity'], self.approach['min_duration'])
            except (RuntimeError, KeyError) as error:
                self._set_phase(ERROR, str(error))
                return TriggerResponse(False, str(error))
            self.approach_attempts = 1
            self._set_phase(APPROACHING, 'moving to the leader pose ({:.2f} rad in {:.1f} s)'
                            .format(distance, duration))
            return TriggerResponse(True, self.message)

    def _stop(self, _request):
        with self.lock:
            self._cancel_goals()
            self._set_phase(IDLE, 'stopped')
            return TriggerResponse(True, self.message)

    def _go_zero(self, _request):
        with self.lock:
            if self.phase in ACTIVE_PHASES:
                return TriggerResponse(False, 'stop before moving to the zero pose')
            goal = {}
            for _, joints, _ in self._controller_entries(self.target):
                for joint in joints:
                    lower, upper = self.limits[joint]
                    goal[joint] = clamp(0.0, lower, upper)
            if not goal:
                return TriggerResponse(False, 'no arm is enabled')
            try:
                self._check_servers(self.target)
                distance, duration = self._slow_move(
                    self.target, goal, self.zero_pose['velocity'], self.zero_pose['min_duration'])
            except RuntimeError as error:
                self._set_phase(ERROR, str(error))
                return TriggerResponse(False, str(error))
            self._set_phase(MOVING_TO_ZERO, 'moving to the zero pose ({:.2f} rad in {:.1f} s)'
                            .format(distance, duration))
            return TriggerResponse(True, self.message)

    # -- powered leader moves ------------------------------------------------------

    def _leader_status_callback(self, msg):
        self.leader_motion = msg.motion

    def _call_leader_trigger(self, service):
        try:
            rospy.wait_for_service(service, timeout=1.0)
            return rospy.ServiceProxy(service, Trigger)()
        except (rospy.ROSException, rospy.ServiceException) as error:
            return TriggerResponse(False, '{} failed: {}'.format(service, error))

    def _move_leader(self, follower_positions, label, wrist_seed=None):
        """Move the leader (under power) to the pose that maps to ``follower_positions``.

        The leader wrist is solved from ``wrist_seed`` (leader angles), else
        from where the leader is now. The seed matters: with the wrist straight
        the leader's forearm roll and handle roll share an axis, so many angle
        pairs give the same hand rotation and the solver keeps the one nearest
        the seed.
        """
        with self.lock:
            if self.phase in ACTIVE_PHASES:
                return TriggerResponse(False, 'stop before moving the leader')
            if self.leader_source != 'feetech':
                return TriggerResponse(False, 'leader source is "{}": nothing to move'.format(
                    self.leader_source))
            leader_now, last = self.leader.snapshot()
            if last is None or not self._leader_alive():
                return TriggerResponse(False, 'no data from the leader arm')
            targets = {}
            for name, arm in self.arms.items():
                if not arm['enabled']:
                    continue
                seed = dict(leader_now)
                seed.update(wrist_seed or {})
                try:
                    pose, wrist_error = follower_to_leader(arm, follower_positions, seed)
                except KeyError as error:
                    return TriggerResponse(False, '{}: no value for {}'.format(name, error))
                if wrist_error > WRIST_SOLVE_TOLERANCE:
                    return TriggerResponse(False, '{}: the leader wrist cannot reach that hand '
                                           'rotation ({:.2f} rad off)'.format(name, wrist_error))
                targets.update(pose)
            targets = {n: v for n, v in targets.items() if n in leader_now}
            if not targets:
                return TriggerResponse(False, 'no enabled arm on the leader')
            distance = max(abs(targets[n] - leader_now[n]) for n in targets)
            duration = max(float(self.leader_move['min_duration']),
                           distance / float(self.leader_move['velocity']))
        try:
            rospy.wait_for_service(self.leader_move['move_service'], timeout=1.0)
            response = rospy.ServiceProxy(self.leader_move['move_service'], MoveLeader)(
                list(targets), [targets[n] for n in targets], duration)
        except (rospy.ROSException, rospy.ServiceException) as error:
            return TriggerResponse(False, 'leader move failed: {}'.format(error))
        if not response.success:
            return TriggerResponse(False, response.message)
        return TriggerResponse(True, 'leader -> {}: {:.2f} rad in {:.1f} s'.format(
            label, distance, duration))

    def _leader_to_init(self, _request):
        pose = {}
        for arm in self.arms.values():
            for joint in [j.follower for j in arm['joints']] + wrist_follower_joints(arm):
                pose[joint] = 0.0
            if arm['gripper'] is not None:
                pose[arm['gripper']['follower']] = arm['gripper']['follower_range'][0]
        pose.update(self.leader_move.get('init_pose') or {})
        # solve from the calibration pose, so the default init pose is exactly
        # the calibration guide (forearm and handle roll 0, not a pair that cancels)
        return self._move_leader(pose, 'init pose', wrist_seed=self.leader_zero)

    def _leader_to_robot(self, _request):
        try:
            positions = self._follower_positions(self.target)
        except RuntimeError as error:
            return TriggerResponse(False, str(error))
        return self._move_leader(positions, '{} robot pose'.format(self.target))

    def _leader_release(self, _request):
        return self._call_leader_trigger(self.leader_move['release_service'])

    def _calibrate_leader(self, _request):
        with self.lock:
            if self.phase in ACTIVE_PHASES:
                return TriggerResponse(False, 'stop before calibrating the leader')
        if self.leader_source != 'feetech':
            return TriggerResponse(False, 'leader source is "{}", nothing to calibrate'
                                   .format(self.leader_source))
        try:
            rospy.wait_for_service(self.calibrate_service, timeout=1.0)
            response = rospy.ServiceProxy(self.calibrate_service, Trigger)()
        except (rospy.ROSException, rospy.ServiceException) as error:
            return TriggerResponse(False, 'calibration failed: {}'.format(error))
        return response

    def _start_recording(self, _request):
        with self.lock:
            try:
                self.recorder.start()
            except (RuntimeError, OSError) as error:
                return TriggerResponse(False, str(error))
            return TriggerResponse(True, 'recording to {}'.format(self.recorder.bag_path))

    def _stop_recording(self, _request):
        with self.lock:
            try:
                path = self.recorder.stop()
            except RuntimeError as error:
                return TriggerResponse(False, str(error))
            return TriggerResponse(True, 'saved {}'.format(path))

    def _set_record_directory(self, request):
        with self.lock:
            try:
                self.recorder.set_directory(request.data)
            except (RuntimeError, OSError) as error:
                return SetStringResponse(False, str(error))
            return SetStringResponse(True, 'recording into {}'.format(self.recorder.directory))

    # -- loop -------------------------------------------------------------------

    def _stream(self, targets, dt):
        """Send the leader pose, rate limited, as a short trajectory."""
        limits = self.follow['max_velocity'][self.target]
        command = {}
        self.speed_limited = []
        for joint, value in targets.items():
            step = float(limits.get(joint, limits['default'])) * dt
            previous = self.last_command.get(joint, value)
            command[joint] = clamp(value, previous - step, previous + step)
            if command[joint] != value:
                self.speed_limited.append(joint)
        self.last_command = command
        lookahead = rospy.Duration(float(self.follow['lookahead']))
        use_action = self.follow.get('interface', 'topic') == 'action'
        for controller, joints, _ in self._controller_entries(self.target):
            trajectory = JointTrajectory()
            trajectory.joint_names = joints
            trajectory.points = [JointTrajectoryPoint(
                positions=[command[j] for j in joints], time_from_start=lookahead)]
            client = self._client(self.target, controller)
            if use_action:
                client.send_goal(FollowJointTrajectoryGoal(trajectory=trajectory))
            else:
                self.command_pubs[(self.target, controller)].publish(trajectory)
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = sorted(command)
        msg.position = [command[n] for n in msg.name]
        self.command_pub.publish(msg)

    def _step(self, dt):
        error = self.recorder.poll()
        if error:
            rospy.logerr(error)
            self.message = error

        if self.phase in (APPROACHING, FOLLOWING) and not self._leader_alive():
            self._cancel_goals()
            self._set_phase(LEADER_LOST, 'leader stopped publishing; holding. Press Start again '
                                         'once it is back')
            return
        try:
            follower = self._follower_positions(self.target)
        except RuntimeError:
            follower = {}
        if self.last_command and follower:
            errors = [abs(v - follower[j]) for j, v in self.last_command.items() if j in follower]
            self.max_tracking_error = max(errors or [0.0])

        if self.phase == APPROACHING:
            state = self._goals_state()
            if state == 'running':
                return
            if state != 'done':
                self._set_phase(ERROR, state)
                return
            targets = self._leader_targets()
            gap = max(abs(targets[j] - follower.get(j, float('inf'))) for j in targets)
            if gap <= self.approach['tolerance']:
                self.last_command = {j: follower[j] for j in targets}
                self._set_phase(FOLLOWING, 'following the leader')
            elif self.approach_attempts >= self.approach['max_attempts']:
                self._set_phase(ERROR, 'still {:.2f} rad from the leader after {} approaches; '
                                'hold the leader still and press Start'.format(
                                    gap, self.approach_attempts))
            else:
                self.approach_attempts += 1
                self._slow_move(self.target, targets, self.approach['velocity'],
                                self.approach['min_duration'])
                self._set_phase(APPROACHING, 'leader moved ({:.2f} rad); approaching again'
                                .format(gap))
        elif self.phase == FOLLOWING:
            self._stream(self._leader_targets(), dt)
            # say which joints the leader drives past the robot's limits
            notes = []
            held_back = self._grippers_held_back()
            if held_back:
                notes.append('gripper not sent to the real robot (send_to_real: false): {}'.format(
                    ', '.join(held_back)))
            if self.clamped:
                notes.append('at joint limit: {}'.format(', '.join(self.clamped)))
            if self.speed_limited:
                notes.append('speed-limited: {}'.format(', '.join(self.speed_limited)))
            self.message = 'following the leader' + (' ({})'.format('; '.join(notes)) if notes else '')
        elif self.phase == MOVING_TO_ZERO:
            state = self._goals_state()
            if state == 'done':
                self._set_phase(IDLE, 'at the zero pose')
            elif state != 'running':
                self._set_phase(ERROR, state)

    def _publish_status(self):
        msg = TeleopStatus()
        msg.header.stamp = rospy.Time.now()
        msg.target = self.target
        msg.real_is_simulated = self.real_is_simulated
        msg.phase = self.phase
        msg.message = self.message
        msg.arms = sorted(self.arms)
        msg.arms_enabled = [self.arms[a]['enabled'] for a in msg.arms]
        msg.leader_source = self.leader_source
        msg.leader_alive = self._leader_alive()
        msg.leader_calibrated = self.leader_calibrated
        msg.leader_rate = self.leader.rate()
        msg.max_tracking_error = self.max_tracking_error
        recorder = self.recorder
        msg.recording = recorder.process is not None
        msg.record_directory = recorder.directory
        if msg.recording:
            msg.current_bag = recorder.bag_path
            msg.recording_duration = rospy.get_time() - recorder.started
            msg.current_bag_size = recorder.current_size()
        msg.bag_count, msg.total_bag_size, msg.disk_free = recorder.directory_stats()
        self.status_pub.publish(msg)

    def spin(self):
        rate_hz = float(self.follow['rate'])
        rate = rospy.Rate(rate_hz)
        status_period = 0.2
        last_status = 0.0
        while not rospy.is_shutdown():
            with self.lock:
                try:
                    self._step(1.0 / rate_hz)
                except (RuntimeError, KeyError) as error:
                    self._cancel_goals()
                    self._set_phase(ERROR, str(error))
                now = rospy.get_time()
                if now - last_status >= status_period:
                    self._publish_status()
                    last_status = now
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def _shutdown(self):
        with self.lock:
            self._cancel_goals()
            if self.recorder.process is not None:
                self.recorder.stop()


def main():
    rospy.init_node('k_arm_teleop_manager')
    TeleopManager().spin()


if __name__ == '__main__':
    main()
