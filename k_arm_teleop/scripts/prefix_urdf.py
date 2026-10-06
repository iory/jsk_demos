#!/usr/bin/env python3
"""Print a URDF with every link name prefixed (used for the virtual robot copy).

Usage: prefix_urdf.py <urdf_file> <prefix>
"""

import sys

from k_arm_teleop.urdf_utils import prefix_links


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    with open(sys.argv[1]) as f:
        sys.stdout.write(prefix_links(f.read(), sys.argv[2]))


if __name__ == '__main__':
    main()
