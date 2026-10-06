"""Small URDF helpers that need nothing but the standard library."""

import xml.etree.ElementTree as ET


MOVABLE_TYPES = ('revolute', 'prismatic', 'continuous')


def parse_joints(urdf_string):
    """Return movable, non-mimic joints of a URDF.

    Parameters
    ----------
    urdf_string : str
        URDF XML.

    Returns
    -------
    dict
        ``{joint_name: (lower, upper)}``. Continuous joints get infinite
        limits. Mimic joints are left out because robot_state_publisher
        computes them itself.
    """
    root = ET.fromstring(urdf_string)
    joints = {}
    for joint in root.findall('joint'):
        if joint.get('type') not in MOVABLE_TYPES or joint.find('mimic') is not None:
            continue
        limit = joint.find('limit')
        if joint.get('type') == 'continuous' or limit is None:
            joints[joint.get('name')] = (float('-inf'), float('inf'))
        else:
            joints[joint.get('name')] = (float(limit.get('lower', 0.0)),
                                         float(limit.get('upper', 0.0)))
    return joints


def prefix_links(urdf_string, prefix):
    """Prefix every link name, so a second copy of a robot gets its own TF frames.

    Joint names are left untouched, so the copy still accepts the same
    trajectories.

    Parameters
    ----------
    urdf_string : str
        URDF XML.
    prefix : str
        Prepended to each link name, e.g. ``'virtual/'``.

    Returns
    -------
    str
        The modified URDF.
    """
    root = ET.fromstring(urdf_string)
    links = {link.get('name') for link in root.findall('link')}
    for link in root.findall('link'):
        link.set('name', prefix + link.get('name'))
    for joint in root.findall('joint'):
        for tag in ('parent', 'child'):
            element = joint.find(tag)
            element.set('link', prefix + element.get('link'))
    for gazebo in root.findall('gazebo'):
        if gazebo.get('reference') in links:
            gazebo.set('reference', prefix + gazebo.get('reference'))
    return ET.tostring(root, encoding='unicode')
