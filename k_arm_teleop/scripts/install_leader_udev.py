#!/usr/bin/env python3
"""Give the leader's USB serial adapter a fixed name, e.g. /dev/k_arm_leader.

Reads the adapter's vendor / product / serial number with udevadm and writes
a udev rule matching exactly that adapter, so the name follows the adapter
whatever ttyACM number or USB port it gets. The rule also opens the device
to the dialout group (MODE 0660), so the user only needs to be in dialout.

    install_leader_udev.py                          # show the rule for the adapter found
    sudo install_leader_udev.py --apply             # write it and reload udev
    install_leader_udev.py --device /dev/ttyACM1 --name k_arm_leader_left

With several USB serial adapters connected, pass --device: the tool refuses
to guess between them.
"""

import argparse
import glob
import os
import subprocess
import sys

RULE_DIR = '/etc/udev/rules.d'


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--device', help='serial device of the adapter (default: the only one)')
    parser.add_argument('--name', default='k_arm_leader',
                        help='name of the link under /dev (default: k_arm_leader)')
    parser.add_argument('--apply', action='store_true',
                        help='write the rule and reload udev (needs root)')
    return parser.parse_args()


def find_device():
    candidates = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    if not candidates:
        sys.exit('no /dev/ttyACM* or /dev/ttyUSB* found: plug the adapter in')
    if len(candidates) > 1:
        sys.exit('several serial devices ({}): pass --device'.format(', '.join(candidates)))
    return candidates[0]


def properties(device):
    out = subprocess.check_output(['udevadm', 'info', '--query=property', '--name', device],
                                  universal_newlines=True)
    props = dict(line.split('=', 1) for line in out.splitlines() if '=' in line)
    missing = [k for k in ('ID_VENDOR_ID', 'ID_MODEL_ID', 'ID_SERIAL_SHORT') if k not in props]
    if missing:
        sys.exit('{} does not report {}; cannot tell it apart from other adapters'.format(
            device, missing))
    return props


def main():
    args = parse_args()
    device = args.device or find_device()
    props = properties(device)
    rule = ('# {name}: leader arm USB adapter ({model}), written by k_arm_teleop install_leader_udev.py\n'
            'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="{vendor}", ATTRS{{idProduct}}=="{product}", '
            'ATTRS{{serial}}=="{serial}", SYMLINK+="{name}", GROUP="dialout", MODE="0660"\n').format(
                name=args.name, model=props.get('ID_MODEL', '?'), vendor=props['ID_VENDOR_ID'],
                product=props['ID_MODEL_ID'], serial=props['ID_SERIAL_SHORT'])
    path = os.path.join(RULE_DIR, '99-{}.rules'.format(args.name))
    print('device: {}\nrule file: {}\n\n{}'.format(device, path, rule))
    if not args.apply:
        print('dry run; run with sudo and --apply to install')
        return
    if os.geteuid() != 0:
        sys.exit('--apply writes {}: run with sudo'.format(RULE_DIR))
    with open(path, 'w') as f:
        f.write(rule)
    subprocess.check_call(['udevadm', 'control', '--reload-rules'])
    subprocess.check_call(['udevadm', 'trigger', '--subsystem-match=tty'])
    subprocess.call(['udevadm', 'settle', '--timeout=5'])
    link = '/dev/' + args.name
    if os.path.exists(link):
        print('ok: {} -> {}'.format(link, os.path.realpath(link)))
    else:
        sys.exit('wrote {}, but {} did not appear; replug the adapter'.format(path, link))


if __name__ == '__main__':
    main()
