#!/usr/bin/env python3
"""Print a URDF with every link name prefixed, optionally without some links.

Usage: prefix_urdf.py <urdf_file> <prefix> [<drop regex>]

The prefix gives a second copy of a robot its own TF frames (the virtual
robot); the drop regex removes links (and their joints) from the display,
e.g. the arm that is not teleoperated.
"""

import sys

from k_arm_teleop.urdf_utils import drop_links
from k_arm_teleop.urdf_utils import prefix_links


def main():
    if len(sys.argv) not in (3, 4):
        sys.exit(__doc__)
    with open(sys.argv[1]) as f:
        urdf = f.read()
    if len(sys.argv) == 4:
        urdf = drop_links(urdf, sys.argv[3])
    sys.stdout.write(prefix_links(urdf, sys.argv[2]) if sys.argv[2] else urdf)


if __name__ == '__main__':
    main()
