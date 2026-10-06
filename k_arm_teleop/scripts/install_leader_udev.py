#!/usr/bin/env python3
"""Give the arms' USB serial adapters fixed names: /dev/k_arm_leader, /dev/k_arm_follower.

Reads each adapter's vendor / product / serial number with udevadm and writes
a udev rule matching exactly that adapter, so the name follows the adapter
whatever ttyACM number or USB port it gets. The rule also opens the device
to the dialout group (MODE 0660), so the user only needs to be in dialout.

Without --role, every connected adapter that has no name yet is pinged for
the leader's servos (ids from config/leader_servos_right.yaml, 1-8): an
adapter where all of them answer is the leader, one where fewer (but some)
answer is the follower. Adapters already named by this tool keep their name.
The servos must be powered and the port free (stop teleop.launch).

    install_leader_udev.py --list                       # adapters and rules now
    install_leader_udev.py                              # show what it would write
    sudo install_leader_udev.py --apply                 # write it
    sudo install_leader_udev.py --role follower --apply # skip the detection
    install_leader_udev.py --role leader --device /dev/ttyACM1 --name k_arm_leader_left

It refuses to move a name to a different adapter without --replace, and to
give an adapter a second name while a rule written by this tool names it
(remove that rule first).
"""

import argparse
import glob
import os
import re
import select
import subprocess
import sys
import termios
import time

RULE_DIR = '/etc/udev/rules.d'
NAMES = {'leader': 'k_arm_leader', 'follower': 'k_arm_follower'}
MARK = 'written by k_arm_teleop install_leader_udev.py'
SERIAL = re.compile(r'ATTRS\{serial\}=="([^"]*)"')
SYMLINK = re.compile(r'SYMLINK\+="([^"]*)"')
CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
BAUDRATES = {1000000: termios.B1000000, 500000: termios.B500000, 115200: termios.B115200}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--role', choices=sorted(NAMES),
                        help='which arm the adapter drives, instead of detecting it: {}'.format(
                            ', '.join('{} -> /dev/{}'.format(k, v) for k, v in sorted(NAMES.items()))))
    parser.add_argument('--device', help='serial device of the adapter (default: every one, or the '
                                         'only one with --role)')
    parser.add_argument('--name', help='name of the link under /dev (default: from the role)')
    parser.add_argument('--baudrate', type=int, default=1000000, choices=sorted(BAUDRATES),
                        help='servo bus speed for the detection')
    parser.add_argument('--apply', action='store_true',
                        help='write the rule(s) and reload udev (needs root)')
    parser.add_argument('--replace', action='store_true',
                        help='overwrite a rule that gives this name to a different adapter')
    parser.add_argument('--list', action='store_true',
                        help='show the connected adapters and the rules this tool wrote')
    args = parser.parse_args()
    if args.name is not None and args.role is None:
        parser.error('--name needs --role')
    return args


def serial_devices():
    return sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))


def find_device():
    candidates = serial_devices()
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


def installed_rules():
    """{path: (name, serial)} for the rules this tool wrote."""
    rules = {}
    for path in sorted(glob.glob(os.path.join(RULE_DIR, '99-*.rules'))):
        try:
            with open(path) as f:
                text = f.read()
        except OSError:
            continue
        if MARK not in text:
            continue
        name, serial = SYMLINK.search(text), SERIAL.search(text)
        rules[path] = (name.group(1) if name else '?', serial.group(1) if serial else '?')
    return rules


def leader_ids():
    """Servo ids of the leader, read from the servo file without a YAML library (sudo's python)."""
    with open(os.path.join(CONFIG, 'leader_servos_right.yaml')) as f:
        ids = sorted({int(i) for i in re.findall(r'\bid:\s*(\d+)', f.read())})
    if not ids:
        sys.exit('no servo ids in {}'.format(os.path.join(CONFIG, 'leader_servos_right.yaml')))
    return ids


def ping(device, ids, baudrate):
    """Ids that answer a FEETECH (SCS protocol) PING on ``device``. Nothing is written to a servo."""
    try:
        fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as error:
        sys.exit('cannot open {}: {} (is teleop.launch using it?)'.format(device, error))
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0  # raw: no input / output / local processing
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = BAUDRATES[baudrate]
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        found = []
        for servo_id in ids:
            termios.tcflush(fd, termios.TCIOFLUSH)
            body = [servo_id, 2, 0x01]  # id, length, PING
            os.write(fd, bytes([0xFF, 0xFF] + body + [~sum(body) & 0xFF]))
            reply = b''
            deadline = time.time() + 0.05
            while len(reply) < 6 and time.time() < deadline:
                if select.select([fd], [], [], deadline - time.time())[0]:
                    reply += os.read(fd, 64)
            start = reply.find(bytes([0xFF, 0xFF, servo_id]))
            if start >= 0 and len(reply) >= start + 6:
                found.append(servo_id)
        return found
    finally:
        os.close(fd)


def show_list():
    rules = installed_rules()
    by_serial = {serial: name for name, serial in rules.values()}
    print('connected adapters:')
    for device in serial_devices():
        serial = properties(device)['ID_SERIAL_SHORT']
        print('  {:<14} serial {:<14} {}'.format(
            device, serial, '/dev/' + by_serial[serial] if serial in by_serial else '(no name)'))
    if not serial_devices():
        print('  (none)')
    print('rules written by this tool:')
    for path, (name, serial) in rules.items():
        present = os.path.exists('/dev/' + name)
        print('  {:<40} /dev/{:<16} serial {:<14} {}'.format(
            path, name, serial, 'connected' if present else 'not connected'))
    if not rules:
        print('  (none)')


def plan_rule(device, role, name, replace):
    """(path, rule text) for naming ``device``, or exit if that would break an existing name."""
    props = properties(device)
    serial = props['ID_SERIAL_SHORT']
    path = os.path.join(RULE_DIR, '99-{}.rules'.format(name))
    for other, (other_name, other_serial) in installed_rules().items():
        if other_serial == serial and other_name != name:
            sys.exit('{} (serial {}) is already /dev/{} by {}.\nIf this adapter really is the '
                     '{}, remove that rule first: sudo rm {}'.format(
                         device, serial, other_name, other, role, other))
        if other == path and other_serial != serial and not replace:
            sys.exit('/dev/{} already belongs to another adapter (serial {}, {}), not to {} '
                     '(serial {}).\nPass --replace if this adapter takes its place.'.format(
                         other_name, other_serial, other, device, serial))
    rule = ('# {name}: {role} arm USB adapter ({model}), {mark}\n'
            'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="{vendor}", ATTRS{{idProduct}}=="{product}", '
            'ATTRS{{serial}}=="{serial}", SYMLINK+="{name}", GROUP="dialout", MODE="0660"\n').format(
                name=name, role=role, model=props.get('ID_MODEL', '?'), mark=MARK,
                vendor=props['ID_VENDOR_ID'], product=props['ID_MODEL_ID'], serial=serial)
    print('{} (serial {}) -> /dev/{}\nrule file: {}{}\n{}'.format(
        device, serial, name, path, ' (replaced)' if os.path.exists(path) else '', rule))
    return path, rule


def detect(devices, baudrate):
    """[(device, role)] for the adapters without a name, from which servos answer."""
    named = {serial for _, serial in installed_rules().values()}
    expected = leader_ids()
    roles = []
    for device in devices:
        serial = properties(device)['ID_SERIAL_SHORT']
        if serial in named:
            print('{} (serial {}): already named, kept'.format(device, serial))
            continue
        found = ping(device, expected, baudrate)
        if found == expected:
            role = 'leader'
        elif found:
            role = 'follower'
        else:
            sys.exit('{} (serial {}): no servo answers at {} bps. Power the servos, free the port, '
                     'or pass --role.'.format(device, serial, baudrate))
        print('{} (serial {}): servos {} answer -> {}'.format(device, serial, found, role))
        roles.append((device, role))
    for role in NAMES:
        same = [d for d, r in roles if r == role]
        if len(same) > 1:
            sys.exit('{} all look like the {}; plug in one at a time or pass --role --device'.format(
                ', '.join(same), role))
    return roles


def main():
    args = parse_args()
    if args.list:
        show_list()
        return
    if args.role is not None:
        targets = [(args.device or find_device(), args.role)]
    else:
        devices = [args.device] if args.device else serial_devices()
        if not devices:
            sys.exit('no /dev/ttyACM* or /dev/ttyUSB* found: plug the adapter in')
        targets = detect(devices, args.baudrate)
        if not targets:
            print('nothing to do: every connected adapter already has a name (see --list)')
            return
    rules = [(name, plan_rule(device, role, name, args.replace))
             for device, role in targets
             for name in [args.name or NAMES[role]]]
    if not args.apply:
        print('dry run; run with sudo and --apply to install')
        return
    if os.geteuid() != 0:
        sys.exit('--apply writes {}: run with sudo'.format(RULE_DIR))
    for _, (path, rule) in rules:
        with open(path, 'w') as f:
            f.write(rule)
    subprocess.check_call(['udevadm', 'control', '--reload-rules'])
    subprocess.check_call(['udevadm', 'trigger', '--subsystem-match=tty'])
    subprocess.call(['udevadm', 'settle', '--timeout=5'])
    missing = []
    for name, (path, _) in rules:
        link = '/dev/' + name
        if os.path.exists(link):
            print('ok: {} -> {}'.format(link, os.path.realpath(link)))
        else:
            missing.append(link)
    if missing:
        sys.exit('rules written, but {} did not appear; replug the adapter'.format(', '.join(missing)))


if __name__ == '__main__':
    main()
