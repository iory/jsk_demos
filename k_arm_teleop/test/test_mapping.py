import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from k_arm_teleop.mapping import build_arm_maps  # noqa: E402
from k_arm_teleop.mapping import counts_to_angle  # noqa: E402
from k_arm_teleop.mapping import leader_to_follower  # noqa: E402
from k_arm_teleop.mapping import leader_zero_pose  # noqa: E402
from k_arm_teleop.urdf_utils import parse_joints  # noqa: E402
from k_arm_teleop.urdf_utils import prefix_links  # noqa: E402


def test_counts_to_angle_wraps_around_center():
    assert counts_to_angle(2048, 2048, 1) == 0.0
    assert math.isclose(counts_to_angle(3072, 2048, 1), math.pi / 2)
    assert math.isclose(counts_to_angle(3072, 2048, -1), -math.pi / 2)
    # 1/4 turn across the 4095 -> 0 boundary
    assert math.isclose(counts_to_angle(1000, 4000, 1), (1096) * 2 * math.pi / 4096)
    # a joint whose range is centred at -2 rad reads -3.5 rad, not +2.78
    angle = counts_to_angle(2048 - 2281, 2048, 1, center=-2.0)
    assert math.isclose(angle, -2281 * 2 * math.pi / 4096)


def test_leader_to_follower_sign_offset_clamp_and_gripper():
    arms = build_arm_maps({'larm': {
        'controller': 'larm_controller',
        'joints': [{'follower': 'A', 'leader': 'a', 'sign': -1.0, 'leader_zero': -0.5}],
        'gripper': {'follower': 'G', 'leader': 'g', 'leader_range': [0.0, 1.0],
                    'follower_range': [0.0, 0.02]}}})
    limits = {'A': (-1.0, 1.0), 'G': (0.0, 0.015)}
    out = leader_to_follower(arms['larm'], {'a': 0.2, 'g': 0.5}, limits, 0.0)
    assert math.isclose(out['arm']['A'], -0.7)
    assert math.isclose(out['gripper']['G'], 0.01)
    out = leader_to_follower(arms['larm'], {'a': -5.0, 'g': 2.0}, limits, 0.1)  # -> +4.5
    assert math.isclose(out['arm']['A'], 0.9)
    assert math.isclose(out['gripper']['G'], 0.015)


URDF = """<robot name="r">
  <link name="base"/><link name="l1"/><link name="l2"/>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/>
    <limit lower="-1" upper="2" effort="1" velocity="1"/></joint>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/>
    <limit lower="-1" upper="2" effort="1" velocity="1"/><mimic joint="j1"/></joint>
  <gazebo reference="l1"/>
</robot>"""


def test_parse_joints_skips_mimic_and_prefix_keeps_joint_names():
    assert parse_joints(URDF) == {'j1': (-1.0, 2.0)}
    prefixed = prefix_links(URDF, 'virtual/')
    assert 'link name="virtual/base"' in prefixed
    assert 'child link="virtual/l1"' in prefixed
    assert 'reference="virtual/l1"' in prefixed
    assert 'joint name="j1"' in prefixed


def test_leader_zero_pose_includes_gripper_release():
    pose = leader_zero_pose({'larm': {
        'joints': [{'follower': 'A', 'leader': 'a', 'leader_zero': 1.5}],
        'gripper': {'leader': 'g', 'leader_range': [0.1, 1.0]}}})
    assert pose == {'a': 1.5, 'g': 0.1}


def test_zyx_split_round_trip_and_wrist_map_zero():
    import numpy as np
    import yaml

    from k_arm_teleop.kinematics import UrdfKinematics
    from k_arm_teleop.kinematics import WristMap
    from k_arm_teleop.kinematics import axis_angle_to_matrix
    from k_arm_teleop.kinematics import rpy_to_matrix
    from k_arm_teleop.kinematics import zyx_angles

    a, b, c = 0.4, -0.7, 1.1
    rot = axis_angle_to_matrix([0, 0, 1], a).dot(axis_angle_to_matrix([0, 1, 0], b)).dot(
        axis_angle_to_matrix([1, 0, 0], c))
    assert np.allclose(zyx_angles(rot), (a, b, c))

    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.join(here, '..', '..')
    leader = UrdfKinematics(open(os.path.join(
        src, 'teleop_leader_pair_description', 'urdf', 'teleop_leader_pair_description.urdf')).read())
    follower = UrdfKinematics(open(os.path.join(
        src, 'k_arm_descriptions', 'urdf', 'K_ARM_DUALARM_primitive.urdf')).read())
    cfg = yaml.safe_load(open(os.path.join(here, '..', 'config', 'teleop.yaml')))
    zero = leader_zero_pose(cfg['arms'])
    for arm in cfg['arms'].values():
        joints = [j['leader'] for j in arm['joints']] + arm['wrist']['leader']
        followers = [j['follower'] for j in arm['joints']] + arm['wrist']['follower']
        wrist = WristMap(leader, follower, joints, followers, zero,
                         rpy_to_matrix(*cfg['leader_base']['rpy']))
        # the calibration pose is the follower's zero pose
        assert np.allclose(list(wrist.follower_wrist(zero).values()), 0.0, atol=1e-6)


def test_follower_to_leader_round_trip():
    import numpy as np
    import yaml

    from k_arm_teleop.kinematics import UrdfKinematics
    from k_arm_teleop.kinematics import WristMap
    from k_arm_teleop.kinematics import rpy_to_matrix
    from k_arm_teleop.mapping import follower_to_leader

    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.join(here, '..', '..')
    leader_urdf = UrdfKinematics(open(os.path.join(
        src, 'teleop_leader_pair_description', 'urdf', 'teleop_leader_pair_description.urdf')).read())
    follower_urdf = open(os.path.join(
        src, 'k_arm_descriptions', 'urdf', 'K_ARM_DUALARM_primitive.urdf')).read()
    limits = parse_joints(follower_urdf)
    cfg = yaml.safe_load(open(os.path.join(here, '..', 'config', 'teleop.yaml')))
    zero = leader_zero_pose(cfg['arms'])
    arms = build_arm_maps(cfg['arms'])
    rng = np.random.default_rng(0)
    for name, arm in arms.items():
        leader_joints = [j.leader for j in arm['joints']] + arm['wrist']['leader']
        follower_joints = [j.follower for j in arm['joints']] + arm['wrist']['follower']
        arm['wrist_map'] = WristMap(leader_urdf, UrdfKinematics(follower_urdf), leader_joints,
                                    follower_joints, zero, rpy_to_matrix(*cfg['leader_base']['rpy']))
        for _ in range(5):
            # a follower pose well inside the limits
            target = {j: 0.5 * float(rng.uniform(*limits[j])) for j in follower_joints}
            target[arm['gripper']['follower']] = 0.006
            seed = dict(zero)
            leader, error = follower_to_leader(arm, target, seed)
            assert error < 1e-6
            arm['wrist_map'].reset()
            back = leader_to_follower(arm, leader, limits, 0.0)
            for j in follower_joints:
                assert abs(back['arm'][j] - target[j]) < 1e-4, (name, j, back['arm'][j], target[j])
            assert abs(back['gripper'][arm['gripper']['follower']] - 0.006) < 1e-9


def test_drop_links_keeps_one_tree():
    import pytest

    from k_arm_teleop.urdf_utils import drop_links

    kept = drop_links(URDF, '^l2$')
    assert 'link name="l2"' not in kept and 'joint name="j2"' not in kept
    assert parse_joints(kept) == {'j1': (-1.0, 2.0)}
    # dropping the middle link splits the tree into two roots
    with pytest.raises(ValueError):
        drop_links(URDF, '^l1$')
    assert drop_links(URDF, '') == URDF


def test_gripper_effort_close_open_then_relax():
    from k_arm_teleop.mapping import gripper_effort

    real = {'open_effort': 0.2, 'close_effort': -0.4, 'open_min_time': 0.3,
            'open_stall_velocity': 0.05, 'open_timeout': 2.0}
    state = {}
    assert math.isclose(gripper_effort(0.5, real, state, 0.0), -0.2)
    assert math.isclose(gripper_effort(1.0, real, state, 0.1), -0.4)
    # let go: push open while it still moves
    assert gripper_effort(0.0, real, state, 1.0, velocity=1.0) == 0.2
    assert gripper_effort(0.0, real, state, 1.2, velocity=0.0) == 0.2  # before open_min_time
    # stopped at the open end: relax and stay relaxed
    assert gripper_effort(0.0, real, state, 1.4, velocity=0.0) == 0.0
    assert gripper_effort(0.0, real, state, 5.0, velocity=0.0) == 0.0
    # pull again, let go again: opens again; without velocity it relaxes on the timeout
    assert gripper_effort(0.8, real, state, 6.0) < 0
    assert gripper_effort(0.0, real, state, 6.1) == 0.2
    assert gripper_effort(0.0, real, state, 8.2) == 0.0


def test_gripper_effort_binary_hysteresis():
    from k_arm_teleop.mapping import gripper_effort

    real = {'open_effort': 0.2, 'close_effort': -0.4, 'mode': 'binary',
            'binary_thresholds': [0.4, 0.6], 'open_timeout': 2.0}
    state = {}
    seen = [gripper_effort(f, real, state, t) for t, f in enumerate((0.5, 0.7, 0.5, 0.3), start=10)]
    # 0.5 from the start: not closed yet -> opening push; 0.7 closes; 0.5 stays closed; 0.3 opens
    assert seen == [0.2, -0.4, -0.4, 0.2]
