"""Leader -> follower joint mapping and the servo count <-> angle conversion."""

import math


COUNTS_PER_REV = 4096


def wrap_counts(delta):
    """Wrap an encoder difference into [-COUNTS_PER_REV / 2, COUNTS_PER_REV / 2)."""
    half = COUNTS_PER_REV // 2
    return (int(delta) + half) % COUNTS_PER_REV - half


def counts_to_angle(raw, zero_raw, direction, center=0.0):
    """Convert a servo reading to a joint angle.

    Parameters
    ----------
    raw : int
        ``present_position`` in encoder counts.
    zero_raw : int
        Reading at which the joint angle is 0 (from calibration).
    direction : int
        +1 when the joint angle grows with the count, -1 otherwise.
    center : float, optional
        The result is the angle equivalent (modulo 2 pi) closest to this
        value, normally the middle of the joint range, so joints whose range
        is wider than pi do not wrap at +-pi.

    Returns
    -------
    float
        Joint angle [rad].
    """
    angle = direction * wrap_counts(raw - zero_raw) * 2.0 * math.pi / COUNTS_PER_REV
    turns = round((center - angle) / (2.0 * math.pi))
    return angle + turns * 2.0 * math.pi


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class _JointMap(object):
    __slots__ = ('follower', 'leader', 'sign', 'leader_zero')

    def __init__(self, follower, leader, sign, leader_zero):
        self.follower = follower
        self.leader = leader
        self.sign = sign
        self.leader_zero = leader_zero


def build_arm_maps(arms_config):
    """Turn the ``arms`` section of teleop.yaml into joint maps.

    Parameters
    ----------
    arms_config : dict
        ``arms`` parameter.

    Returns
    -------
    dict
        ``{arm: {'controller', 'enabled', 'joints': [_JointMap], 'gripper': dict or None}}``
    """
    arms = {}
    for name, cfg in sorted(arms_config.items()):
        joints = [_JointMap(j['follower'], j['leader'], float(j.get('sign', 1.0)),
                            float(j.get('leader_zero', 0.0)))
                  for j in cfg['joints']]
        gripper = cfg.get('gripper')
        if gripper is not None and not gripper.get('enabled', True):
            gripper = None
        arms[name] = {
            'controller': cfg['controller'],
            'enabled': bool(cfg.get('enabled', True)),
            'joints': joints,
            'wrist': cfg.get('wrist'),
            # set by the caller: kinematics.WristMap built from both URDFs
            'wrist_map': None,
            'gripper': gripper,
        }
    return arms


def leader_zero_pose(arms_config):
    """Leader URDF angles of the calibration pose, i.e. where the follower reads 0.

    Parameters
    ----------
    arms_config : dict
        ``arms`` parameter of teleop.yaml.

    Returns
    -------
    dict
        ``{leader_joint: angle}``; a gripper's entry is the released end of
        its ``leader_range``.
    """
    pose = {}
    for cfg in arms_config.values():
        for joint in cfg['joints']:
            pose[joint['leader']] = float(joint.get('leader_zero', 0.0))
        wrist = cfg.get('wrist')
        if wrist is not None:
            pose.update(zip(wrist['leader'], [float(v) for v in wrist['leader_zero']]))
        gripper = cfg.get('gripper')
        if gripper is not None:
            pose[gripper['leader']] = float(gripper['leader_range'][0])
    return pose


def wrist_follower_joints(arm):
    """Follower joint names driven through the wrist map (empty without one)."""
    return list(arm['wrist']['follower']) if arm.get('wrist') else []


def leader_to_follower(arm, leader_positions, limits, margin, previous=None):
    """Map leader joint angles of one arm to follower targets.

    Arm joints: ``follower = sign * (leader - leader_zero)``. Wrist joints
    (``arm['wrist']``): the hand rotation is matched through
    ``arm['wrist_map']`` (see kinematics.WristMap).

    Parameters
    ----------
    arm : dict
        One entry of :func:`build_arm_maps`.
    leader_positions : dict
        ``{leader_joint: angle}``.
    limits : dict
        Follower ``{joint: (lower, upper)}``.
    margin : float
        Targets are kept this far inside the limits.

    Returns
    -------
    dict
        ``{'arm': {follower_joint: value}, 'gripper': {follower_joint: value},
        'clamped': [follower joints whose target was cut at a limit]}``.

    Raises
    ------
    KeyError
        If a leader joint of this arm has not been received.
    """
    arm_targets = {}
    clamped = []
    for joint in arm['joints']:
        value = joint.sign * (leader_positions[joint.leader] - joint.leader_zero)
        lower, upper = limits[joint.follower]
        arm_targets[joint.follower] = clamp(value, lower + margin, upper - margin)
        if arm_targets[joint.follower] != value:
            clamped.append(joint.follower)
    if arm.get('wrist'):
        if arm['wrist_map'] is None:
            raise RuntimeError('arm has a wrist section but no wrist_map was built')
        missing = [j for j in arm['wrist']['leader'] if j not in leader_positions]
        if missing:
            raise KeyError(missing[0])
        wrist = arm['wrist_map'].follower_wrist(leader_positions, limits, previous)
        for name, value in wrist.items():
            lower, upper = limits[name]
            arm_targets[name] = clamp(value, lower + margin, upper - margin)
            if arm_targets[name] != value:
                clamped.append(name)
    gripper_targets = {}
    gripper = arm['gripper']
    if gripper is not None:
        l0, l1 = gripper['leader_range']
        f0, f1 = gripper['follower_range']
        ratio = clamp((leader_positions[gripper['leader']] - l0) / (l1 - l0), 0.0, 1.0)
        value = f0 + ratio * (f1 - f0)
        lower, upper = limits[gripper['follower']]
        gripper_targets[gripper['follower']] = clamp(value, lower, upper)
    return {'arm': arm_targets, 'gripper': gripper_targets, 'clamped': clamped}
