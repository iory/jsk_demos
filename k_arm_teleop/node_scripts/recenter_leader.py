#!/usr/bin/env python3
"""Shift each leader servo's encoder seam out of the joint's range.

A FEETECH servo in position mode takes absolute goals 0..4095. When a
joint's range crosses the 0/4095 seam, a powered move across it makes the
servo turn the long way round, into its mechanical stop, so
feetech_leader_driver.py refuses such moves. This tool rewrites each
servo's ``homing_offset`` (EEPROM) so the middle of the joint's range reads
2048, and converts the counts stored in the calibration file (zero_raw,
trigger points) by the same amount, so the calibration stays valid.

Dry run by default; ``--apply`` writes. Stop teleop.launch first: the tool
needs the serial bus to itself.

    rosrun k_arm_teleop recenter_leader.py --arm right            # show the plan
    rosrun k_arm_teleop recenter_leader.py --arm right --apply    # write EEPROM + calibration
"""

import argparse
import os
import sys
import time

from feetech_cli import FeetechServoController
import rospkg
import yaml

COUNTS = 4096
CENTER = COUNTS // 2
SEAM_MARGIN = 20  # [count] keep the range this far from 0 and 4095
MAX_OFFSET = 2047  # homing_offset register limit (sign bit 11)


def parse_args():
    config = os.path.join(rospkg.RosPack().get_path('k_arm_teleop'), 'config')
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--arm', choices=('right', 'left'), default='right')
    parser.add_argument('--bus', default=os.path.join(config, 'leader_bus.yaml'))
    parser.add_argument('--servos', help='default: config/leader_servos_<arm>.yaml')
    parser.add_argument('--calibration', default=os.path.join(config, 'leader_calibration.yaml'))
    parser.add_argument('--teleop', default=os.path.join(config, 'teleop.yaml'))
    parser.add_argument('--apply', action='store_true', help='write the servos and the file')
    parser.add_argument('--yes', action='store_true', help='do not ask before writing')
    args = parser.parse_args()
    if args.servos is None:
        args.servos = os.path.join(config, 'leader_servos_{}.yaml'.format(args.arm))
    return args


def load(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}


def wrap(delta):
    return (int(delta) + CENTER) % COUNTS - CENTER


def raw_range(joint, calibration, ranges, leader_zero):
    """(low, high) counts the joint covers, unwrapped (may go below 0 or above 4095)."""
    trigger = (calibration.get('trigger') or {}).get(joint)
    if trigger and 'released' in trigger and 'pulled' in trigger:
        start = trigger['released']
        end = start + wrap(trigger['pulled'] - start)
        return min(start, end), max(start, end)
    zero = calibration['zero_raw'][joint]
    direction = calibration['direction'][joint]
    if joint not in ranges:
        raise KeyError('{} has no joint_ranges_deg entry'.format(joint))
    per_deg = COUNTS / 360.0
    ends = [zero + direction * deg * per_deg for deg in ranges[joint]]
    return min(ends), max(ends)


def shift_servo(bus, servo_id, shift):
    """Make the servo read ``shift`` counts less (mod 4096) in every pose.

    present_position = (raw_encoder - homing_offset) mod 4096 (STS3215), so
    the offset grows by ``shift``. The register holds +-2047; any shift has an
    equivalent offset in that range modulo 4096, which is written here
    (feetech-cli's set_zero does not reduce it and would refuse some of them).
    """
    offset = bus.read_register(servo_id, 'homing_offset')
    present = bus.read_register(servo_id, 'present_position') % COUNTS
    target = (present - shift) % COUNTS
    new_offset = wrap(offset + shift)
    if abs(new_offset) > MAX_OFFSET:
        # -2048 has no positive twin within +-2047: settle one count off
        new_offset += 1 if new_offset < 0 else -1
        target = (present + offset - new_offset) % COUNTS
    bus.set_torque(servo_id, False)
    bus.write_register(servo_id, 'homing_offset', new_offset)
    time.sleep(0.05)
    reported = bus.read_register(servo_id, 'present_position') % COUNTS
    if abs(wrap(reported - target)) > 2:
        sys.exit('servo {}: expected to read {} after the offset change, reads {}; stopping'.format(
            servo_id, target, reported))
    # keep the goal in the new frame, then leave the torque off (the leader is moved by hand)
    bus.set_goal_position(servo_id, reported)
    bus.set_torque(servo_id, False)
    return present, reported


def main():
    args = parse_args()
    bus_config = load(args.bus)
    servos = load(args.servos)['servos']
    calibration = load(args.calibration)
    ranges = bus_config.get('joint_ranges_deg', {})
    missing = [s['joint'] for s in servos if s['joint'] not in (calibration.get('zero_raw') or {})]
    if missing:
        sys.exit('calibrate the leader first; no zero_raw for {}'.format(missing))

    plan = []
    print('{:<14} {:>3} {:>16} {:>7} {:>16}'.format('joint', 'id', 'range now', 'shift', 'range after'))
    for servo in servos:
        joint = servo['joint']
        low, high = raw_range(joint, calibration, ranges, None)
        shift = int(round((low + high) / 2.0 - CENTER))
        after = (low - shift, high - shift)
        crosses = low < SEAM_MARGIN or high > COUNTS - 1 - SEAM_MARGIN
        fits = after[0] >= SEAM_MARGIN and after[1] <= COUNTS - 1 - SEAM_MARGIN
        print('{:<14} {:>3} {:>7.0f}..{:<7.0f} {:>+7d} {:>7.0f}..{:<7.0f} {}'.format(
            joint, servo['id'], low, high, shift, after[0], after[1],
            ('crosses the seam' if crosses else 'ok') + ('' if fits else '  RANGE TOO WIDE')))
        if not fits:
            sys.exit('{} covers more than one turn minus the margin; cannot recenter'.format(joint))
        plan.append((servo, shift))

    if not args.apply:
        print('\ndry run; add --apply to write homing_offset (EEPROM) and {}'.format(args.calibration))
        return
    if not args.yes:
        answer = input('\nWrite homing_offset to the EEPROM of servos {} and update {}? [y/N] '.format(
            [s['id'] for s, _ in plan], args.calibration))
        if answer.strip().lower() != 'y':
            print('nothing written')
            return

    port = bus_config.get('port') or None
    baudrate = bus_config.get('baudrate') or None
    with FeetechServoController(port=port, baudrate=baudrate) as bus:
        found = bus.scan()
        absent = [s['id'] for s, _ in plan if s['id'] not in found]
        if absent:
            sys.exit('servos {} do not answer; nothing written'.format(absent))
        for servo, shift in plan:
            if shift == 0:
                continue
            before, after = shift_servo(bus, servo['id'], shift)
            print('servo {} ({}): reads {} -> {}'.format(servo['id'], servo['joint'], before, after))

    for servo, shift in plan:
        joint = servo['joint']
        calibration['zero_raw'][joint] = (calibration['zero_raw'][joint] - shift) % COUNTS
        trigger = (calibration.get('trigger') or {}).get(joint)
        if trigger:
            for point in list(trigger):
                trigger[point] = (trigger[point] - shift) % COUNTS
    with open(args.calibration, 'w') as f:
        f.write('# feetech_leader_driver.py\n'
                '# zero_raw: encoder count at the calibration pose.\n'
                '# direction: +1 if the URDF joint angle grows with the count, -1 otherwise.\n'
                '# trigger: counts with the gripper trigger released / pulled.\n')
        yaml.safe_dump(calibration, f, default_flow_style=False)
    print('updated {}'.format(args.calibration))


if __name__ == '__main__':
    main()
