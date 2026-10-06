"""Small URDF helpers that need nothing but the standard library."""

import re
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


def drop_links(urdf_string, pattern):
    """Remove links whose name matches ``pattern`` (regex), and every joint touching them.

    For display: the remaining links must still form one tree (e.g. dropping
    the linear stage and one arm leaves the other arm, rooted at its first
    link); rviz places each link by TF anyway.

    Parameters
    ----------
    urdf_string : str
        URDF XML.
    pattern : str
        Regular expression searched in link names; empty keeps everything.

    Returns
    -------
    str
        The URDF without those links.

    Raises
    ------
    ValueError
        If what is left is not a single tree.
    """
    root = ET.fromstring(urdf_string)
    if not pattern:
        return urdf_string
    regex = re.compile(pattern)
    dropped = {link.get('name') for link in root.findall('link') if regex.search(link.get('name'))}
    for element in list(root):
        if element.tag == 'link' and element.get('name') in dropped:
            root.remove(element)
        elif element.tag == 'joint' and ({element.find('parent').get('link'),
                                          element.find('child').get('link')} & dropped):
            root.remove(element)
        elif element.tag == 'gazebo' and element.get('reference') in dropped:
            root.remove(element)
    links = {link.get('name') for link in root.findall('link')}
    children = {joint.find('child').get('link') for joint in root.findall('joint')}
    roots = links - children
    if len(roots) != 1:
        raise ValueError('dropping {!r} leaves {} roots: {}'.format(pattern, len(roots), sorted(roots)))
    return ET.tostring(root, encoding='unicode')
