#!/usr/bin/env python3
"""Publish the leader arm's joint angles read from FEETECH servos.

Reads ``present_position`` of every servo in ``~servos`` with feetech-cli
and publishes ``joint_states`` in the leader URDF's joint names, only while
every servo answers. ``servo_status`` reports each servo (connected,
position, voltage, temperature, read errors) for the rviz panel; missing
servos and a missing port are retried every second instead of exiting.

Calibration: hold the leader in the calibration pose (arm hanging down,
see teleop.yaml ``leader_zero``) and call ``calibrate_zero``
(std_srvs/Trigger). The current counts are stored in ``~calibration_file``;
nothing is written to the servos. Until a calibration exists, ``calibrated``
(latched std_msgs/Bool) is False and count 2048 stands for the calibration
pose.
"""

import math
import os
import threading

from feetech_cli import FeetechError
from feetech_cli import FeetechServoController
from feetech_cli.registers import CENTER_POSITION
import rospy
import serial
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse
import yaml

from k_arm_teleop.mapping import counts_to_angle
from k_arm_teleop.mapping import wrap_counts
from k_arm_teleop.mapping import leader_zero_pose
from k_arm_teleop.msg import LeaderServoStatus
from k_arm_teleop.msg import ServoState
from k_arm_teleop.srv import SetServoDirection
from k_arm_teleop.srv import SetServoDirectionResponse
from k_arm_teleop.srv import SetString
from k_arm_teleop.srv import SetStringResponse
from k_arm_teleop.urdf_utils import parse_joints


RETRY_PERIOD = 1.0     # [s] between attempts to reopen the bus / reach a missing servo
STATUS_PERIOD = 0.2    # [s] between servo_status messages
MIN_TRIGGER_TRAVEL = 100  # [count] released and pulled must be at least this far apart
RANGE_MARGIN = 0.2     # [rad] beyond the physical range before a joint is flagged


class FeetechLeaderDriver(object):

    def __init__(self):
        self.servos = rospy.get_param('~servos')
        self.calibration_file = os.path.expanduser(rospy.get_param('~calibration_file'))
        limits = parse_joints(rospy.get_param('robot_description'))
        unknown = [s['joint'] for s in self.servos if s['joint'] not in limits]
        if unknown:
            raise RuntimeError('joints {} are not in the leader robot_description'.format(unknown))
        # Leader URDF angles of the calibration pose (arm hanging down), from
        # the arms section of teleop.yaml. A calibrated joint reads this angle
        # at its stored zero count.
        zero_pose = leader_zero_pose(rospy.get_param('~arms'))
        self.calibration_pose = {s['joint']: zero_pose.get(s['joint'], 0.0) for s in self.servos}
        # The first reading of a joint is taken within +-pi of the middle of its
        # physical range, so the +-pi seam lies outside the range the joint can
        # reach. After that the angle is tracked continuously.
        # Range of each joint in the published angle convention: calibration
        # pose + joint_ranges_deg (leader_bus.yaml, relative to the hanging
        # pose), else the URDF limits.
        ranges = rospy.get_param('~joint_ranges_deg', {})
        self.ranges = {}
        for servo in self.servos:
            joint = servo['joint']
            if joint in ranges:
                cal = self.calibration_pose[joint]
                lower, upper = (math.radians(v) for v in ranges[joint])
                self.ranges[joint] = (cal + lower, cal + upper)
            else:
                self.ranges[joint] = limits[joint]
        self.range_centers = {joint: 0.5 * (lower + upper)
                              for joint, (lower, upper) in self.ranges.items()}

        self.zero_raw = {}
        # Gripper triggers: leader_range from teleop.yaml, and the counts recorded
        # with the trigger released and pulled (calibrate_trigger).
        self.trigger_ranges = {}
        for arm in rospy.get_param('~arms').values():
            gripper = arm.get('gripper')
            if gripper is not None:
                self.trigger_ranges[gripper['leader']] = tuple(gripper['leader_range'])
        self.trigger_points = {}
        # leader_bus.yaml gives the defaults; directions flipped from the panel
        # are stored in the calibration file and win over them.
        self.directions = {s['joint']: int(s.get('direction', 1)) for s in self.servos}
        self.calibrated = self._load_calibration()

        self.port = rospy.get_param('~port', '') or None
        self.baudrate = rospy.get_param('~baudrate', 1000000) or None
        self.torque_off = rospy.get_param('~torque_off_on_start', True)
        self.bus = None
        self.bus_error = 'not opened yet'

        self.lock = threading.Lock()
        self.latest_raw = None
        # Per servo: what the panel shows. A servo that stops answering is
        # pinged again every RETRY_PERIOD and picked up when it comes back.
        self.state = {s['id']: {'connected': False, 'tracking': False, 'raw': 0, 'angle': 0.0,
                                'voltage': 0.0,
                                'temperature': 0, 'errors': 0, 'last_error': 'not found yet'}
                      for s in self.servos}

        self.status_pub = rospy.Publisher('servo_status', LeaderServoStatus, queue_size=1)
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.calibrated_pub = rospy.Publisher('calibrated', Bool, queue_size=1, latch=True)
        self.calibrated_pub.publish(Bool(data=self.calibrated))
        rospy.Service('calibrate_zero', Trigger, self._calibrate)
        rospy.Service('set_direction', SetServoDirection, self._set_direction)
        rospy.Service('calibrate_trigger', SetString, self._calibrate_trigger)

    def _load_calibration(self):
        if not os.path.exists(self.calibration_file):
            rospy.logwarn('no leader calibration at %s: angles are relative to count %d '
                          'until "calibrate_zero" is called', self.calibration_file,
                          CENTER_POSITION)
            return False
        with open(self.calibration_file) as f:
            data = yaml.safe_load(f) or {}
        for joint, direction in (data.get('direction') or {}).items():
            if joint in self.directions:
                self.directions[joint] = int(direction)
        for joint, points in (data.get('trigger') or {}).items():
            if joint in self.trigger_ranges:
                self.trigger_points[joint] = {k: int(v) for k, v in points.items()}
        zero_raw = data.get('zero_raw', {})
        missing = [s['joint'] for s in self.servos if s['joint'] not in zero_raw]
        if missing:
            rospy.logwarn('calibration %s has no entry for %s; treating the leader as '
                          'uncalibrated', self.calibration_file, missing)
            return False
        self.zero_raw = {name: int(value) for name, value in zero_raw.items()}
        rospy.loginfo('loaded leader calibration from %s', self.calibration_file)
        return True

    def _calibrate(self, _request):
        with self.lock:
            raw = None if self.latest_raw is None else dict(self.latest_raw)
        if raw is None:
            missing = [s['id'] for s in self.servos if not self.state[s['id']]['connected']]
            return TriggerResponse(success=False, message='servos {} are not answering'.format(
                missing))
        with self.lock:
            self.zero_raw = raw
            self.calibrated = True
        for state in self.state.values():
            state['tracking'] = False
        self._save()
        self.calibrated_pub.publish(Bool(data=True))
        rospy.loginfo('leader zero calibrated: %s -> %s', raw, self.calibration_file)
        return TriggerResponse(success=True, message='saved to {}'.format(self.calibration_file))

    def _save(self):
        """Write this driver's joints into the calibration file, keeping every other joint.

        One file serves both arms (arm:=right / arm:=left), so entries of joints
        this driver does not own are left as they are.
        """
        directory = os.path.dirname(self.calibration_file)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        data = {}
        if os.path.exists(self.calibration_file):
            with open(self.calibration_file) as f:
                data = yaml.safe_load(f) or {}
        with self.lock:
            data.setdefault('direction', {}).update(self.directions)
            if self.calibrated:
                data.setdefault('zero_raw', {}).update(self.zero_raw)
            if self.trigger_points:
                trigger = data.setdefault('trigger', {})
                for joint, points in self.trigger_points.items():
                    trigger[joint] = dict(points)
        with open(self.calibration_file, 'w') as f:
            f.write('# feetech_leader_driver.py\n'
                    '# zero_raw: encoder count at the calibration pose.\n'
                    '# direction: +1 if the URDF joint angle grows with the count, -1 otherwise.\n'
                    '# trigger: counts with the gripper trigger released / pulled.\n')
            yaml.safe_dump(data, f, default_flow_style=False)

    def _calibrate_trigger(self, request):
        """Record the current count of a trigger as "<joint> released" or "<joint> pulled"."""
        parts = request.data.split()
        if len(parts) != 2 or parts[1] not in ('released', 'pulled'):
            return SetStringResponse(False, 'expected "<joint> released" or "<joint> pulled"')
        joint, point = parts
        if joint not in self.trigger_ranges:
            return SetStringResponse(False, '{} is not a gripper trigger'.format(joint))
        servo = next((s for s in self.servos if s['joint'] == joint), None)
        if servo is None or not self.state[servo['id']]['connected']:
            return SetStringResponse(False, '{} is not answering'.format(joint))
        with self.lock:
            self.trigger_points.setdefault(joint, {})[point] = self.state[servo['id']]['raw']
            points = dict(self.trigger_points[joint])
        if 'released' in points and 'pulled' in points and \
                abs(wrap_counts(points['pulled'] - points['released'])) < MIN_TRIGGER_TRAVEL:
            return SetStringResponse(False, 'released and pulled are only {} counts apart; '
                                     'pull the trigger all the way'.format(
                                         abs(wrap_counts(points['pulled'] - points['released']))))
        self._save()
        missing = [p for p in ('released', 'pulled') if p not in points]
        return SetStringResponse(True, '{} {} = {}{}'.format(
            joint, point, points[point],
            '' if not missing else ' (still needs: {})'.format(missing[0])))

    def _trigger_angle(self, joint, value):
        """Linear map of a trigger count between its released and pulled counts."""
        points = self.trigger_points[joint]
        lower, upper = self.trigger_ranges[joint]
        travel = wrap_counts(points['pulled'] - points['released'])
        return lower + (upper - lower) * wrap_counts(value - points['released']) / float(travel)

    def _trigger_ready(self, joint):
        points = self.trigger_points.get(joint, {})
        return 'released' in points and 'pulled' in points

    def _set_direction(self, request):
        if request.joint not in self.directions:
            return SetServoDirectionResponse(False, 'unknown joint {}'.format(request.joint))
        if request.direction not in (1, -1):
            return SetServoDirectionResponse(False, 'direction must be +1 or -1')
        with self.lock:
            self.directions[request.joint] = int(request.direction)
        for servo in self.servos:
            if servo['joint'] == request.joint:
                self.state[servo['id']]['tracking'] = False
        self._save()
        rospy.loginfo('%s direction set to %+d', request.joint, request.direction)
        return SetServoDirectionResponse(True, '{} direction {:+d} (saved to {})'.format(
            request.joint, request.direction, self.calibration_file))

    def _open_bus(self):
        try:
            bus = FeetechServoController(port=self.port, baudrate=self.baudrate)
            found = bus.open(require_servo=False)
        except (FeetechError, OSError) as error:
            self.bus_error = str(error)
            rospy.logerr_throttle(5.0, 'leader bus: %s', error)
            return
        self.bus = bus
        self.bus_error = ''
        rospy.loginfo('leader bus %s @ %d: answering %s', bus.serial.port, bus.baudrate, found)

    def _connect(self, servo):
        """Ping a missing servo; on success switch its torque off if asked to."""
        state = self.state[servo['id']]
        try:
            if not self.bus.ping(servo['id']):
                state['last_error'] = 'no answer to ping'
                return
            if self.torque_off:
                self.bus.set_torque(servo['id'], False)
        except FeetechError as error:
            state['last_error'] = str(error)
            return
        state['connected'] = True
        state['last_error'] = ''
        rospy.loginfo('servo %d (%s) connected', servo['id'], servo['joint'])

    def _read_health(self, servo):
        state = self.state[servo['id']]
        try:
            state['voltage'] = self.bus.read_register(servo['id'], 'present_voltage') / 10.0
            state['temperature'] = self.bus.read_register(servo['id'], 'present_temperature')
        except FeetechError as error:
            self._mark_failed(servo, error)

    def _drop_bus(self, error):
        rospy.logerr('leader bus lost: %s', error)
        try:
            self.bus.close()
        except (serial.SerialException, OSError):
            pass
        self.bus = None
        self.bus_error = str(error)
        for servo in self.servos:
            self._mark_failed(servo, error)

    def _mark_failed(self, servo, error):
        state = self.state[servo['id']]
        if state['connected']:
            rospy.logwarn('servo %d (%s) stopped answering: %s', servo['id'], servo['joint'], error)
        state['connected'] = False
        # the joint may have turned a lot while it was not answering
        state['tracking'] = False
        state['errors'] += 1
        state['last_error'] = str(error)

    def _published_angle(self, servo):
        """Angle for servo_joint_states: held inside the range for clamp_to_range joints."""
        angle = self.state[servo['id']]['angle']
        if servo.get('clamp_to_range', False):
            lower, upper = self.ranges[servo['joint']]
            angle = max(lower, min(upper, angle))
        return angle

    def _out_of_range(self, joint, angle):
        lower, upper = self.ranges[joint]
        return angle < lower - RANGE_MARGIN or angle > upper + RANGE_MARGIN

    def _publish_status(self):
        msg = LeaderServoStatus()
        msg.header.stamp = rospy.Time.now()
        msg.port = self.bus.serial.port if self.bus is not None else (self.port or 'auto')
        msg.bus_error = self.bus_error
        msg.calibrated = self.calibrated
        msg.all_connected = all(st['connected'] for st in self.state.values())
        for servo in self.servos:
            st = self.state[servo['id']]
            msg.servos.append(ServoState(
                id=servo['id'], joint=servo['joint'], connected=st['connected'],
                direction=self.directions[servo['joint']],
                out_of_range=self._out_of_range(servo['joint'], st['angle']) if st['tracking'] else False,
                trigger_calibrated=self._trigger_ready(servo['joint']),
                raw_position=st['raw'], angle=st['angle'], voltage=st['voltage'],
                temperature=st['temperature'], read_errors=st['errors'],
                last_error=st['last_error']))
        self.status_pub.publish(msg)

    def spin(self):
        rate = rospy.Rate(rospy.get_param('~rate', 100.0))
        names = [s['joint'] for s in self.servos]
        last_retry = last_status = last_health = 0.0
        health_index = 0
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if self.bus is None:
                if now - last_retry >= RETRY_PERIOD:
                    last_retry = now
                    self._open_bus()
            if self.bus is not None:
                if now - last_retry >= RETRY_PERIOD:
                    last_retry = now
                    for servo in self.servos:
                        if not self.state[servo['id']]['connected']:
                            self._connect(servo)
                raw = {}
                for servo in self.servos:
                    state = self.state[servo['id']]
                    if not state['connected']:
                        continue
                    try:
                        value = self.bus.read_register(servo['id'], 'present_position') % 4096
                    except FeetechError as error:
                        self._mark_failed(servo, error)
                        continue
                    except (serial.SerialException, OSError) as error:
                        # adapter unplugged: drop the port and reopen it later
                        self._drop_bus(error)
                        break
                    with self.lock:
                        zero = self.zero_raw.get(servo['joint'], CENTER_POSITION)
                    state['raw'] = value
                    # Track the angle continuously (multi-turn) from the last reading,
                    # so a joint never jumps by 2 pi.
                    cal = self.calibration_pose[servo['joint']]
                    reference = state['angle'] if state['tracking'] else \
                        self.range_centers[servo['joint']]
                    if self._trigger_ready(servo['joint']):
                        state['angle'] = self._trigger_angle(servo['joint'], value)
                    else:
                        state['angle'] = cal + counts_to_angle(
                            value, zero, self.directions[servo['joint']], center=reference - cal)
                    state['tracking'] = True
                    raw[servo['joint']] = value
                # one servo's voltage / temperature at a time, ~1 s per servo
                if now - last_health >= RETRY_PERIOD / len(self.servos):
                    last_health = now
                    servo = self.servos[health_index % len(self.servos)]
                    health_index += 1
                    if self.state[servo['id']]['connected']:
                        self._read_health(servo)
                if len(raw) == len(self.servos):
                    # Publish only complete poses; a partial or stale one would
                    # be followed by the robot.
                    with self.lock:
                        self.latest_raw = raw
                    msg = JointState()
                    msg.header.stamp = rospy.Time.now()
                    msg.name = names
                    msg.position = [self._published_angle(s) for s in self.servos]
                    self.joint_pub.publish(msg)
                else:
                    with self.lock:
                        self.latest_raw = None
            if now - last_status >= STATUS_PERIOD:
                last_status = now
                self._publish_status()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
        if self.bus is not None:
            self.bus.close()


def main():
    rospy.init_node('feetech_leader_driver')
    FeetechLeaderDriver().spin()


if __name__ == '__main__':
    main()
