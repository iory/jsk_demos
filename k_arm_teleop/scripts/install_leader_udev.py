#!/usr/bin/env python3
"""Give the arms' USB serial adapters fixed names: /dev/k_arm_leader, /dev/k_arm_follower.

Reads each adapter's vendor / product / serial number with udevadm and writes
a udev rule matching exactly that adapter, so the name follows the adapter
whatever ttyACM number or USB port it gets. The rule also opens the device
to the dialout group (MODE 0660), so the user only needs to be in dialout.

Which adapter is which comes from config/usb_adapters.yaml (serial number ->
role): the leader and the follower carry the same servos, so nothing on the
bus tells them apart. Without --role, every connected adapter listed there
is named; adapters already named by this tool keep their name. A new adapter
needs --role once; --apply then also adds it to usb_adapters.yaml (commit
that, and other PCs know it too).

    install_leader_udev.py --list                       # adapters and rules now
    install_leader_udev.py                              # show what it would write
    sudo install_leader_udev.py --apply                 # write it
    sudo install_leader_udev.py --role follower --apply # a new adapter
    install_leader_udev.py --role leader --device /dev/ttyACM1 --name k_arm_leader_left

It refuses to move a name to a different adapter without --replace, and to
give an adapter a second name while a rule written by this tool names it
(remove that rule first).
"""

import argparse
import glob
import os
import re
import subprocess
import sys

RULE_DIR = '/etc/udev/rules.d'
NAMES = {'leader': 'k_arm_leader', 'follower': 'k_arm_follower'}
MARK = 'written by k_arm_teleop install_leader_udev.py'
SERIAL = re.compile(r'ATTRS\{serial\}=="([^"]*)"')
SYMLINK = re.compile(r'SYMLINK\+="([^"]*)"')
ADAPTERS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config',
                                         'usb_adapters.yaml'))
ADAPTER_LINE = re.compile(r'^\s*["\']?([^"\'\s:#]+)["\']?\s*:\s*(\w+)\s*(#.*)?$')


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--role', choices=sorted(NAMES),
                        help='which arm the adapter drives, for an adapter not in {}: {}'.format(
                            os.path.basename(ADAPTERS),
                            ', '.join('{} -> /dev/{}'.format(k, v) for k, v in sorted(NAMES.items()))))
    parser.add_argument('--device', help='serial device of the adapter (default: every one, or the '
                                         'only one with --role)')
    parser.add_argument('--name', help='name of the link under /dev (default: from the role)')
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


def known_adapters():
    """{serial: role} from usb_adapters.yaml, read without a YAML library (sudo's python)."""
    adapters = {}
    if not os.path.exists(ADAPTERS):
        return adapters
    with open(ADAPTERS) as f:
        for number, line in enumerate(f, 1):
            if not line.strip() or line.lstrip().startswith('#'):
                continue
            match = ADAPTER_LINE.match(line)
            if match is None or match.group(2) not in NAMES:
                sys.exit('{}:{}: expected "<serial>: {}"'.format(
                    ADAPTERS, number, '|'.join(sorted(NAMES))))
            adapters[match.group(1)] = match.group(2)
    return adapters


def remember_adapter(serial, role):
    """Add ``serial: role`` to usb_adapters.yaml, keeping the file's owner (written under sudo)."""
    stat = os.stat(ADAPTERS) if os.path.exists(ADAPTERS) else None
    with open(ADAPTERS, 'a') as f:
        f.write('{}: {}\n'.format(serial, role))
    if stat is not None:
        os.chown(ADAPTERS, stat.st_uid, stat.st_gid)
    print('added "{}: {}" to {}; commit it so other PCs know this adapter'.format(
        serial, role, os.path.normpath(ADAPTERS)))


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


def show_list():
    rules = installed_rules()
    by_serial = {serial: name for name, serial in rules.values()}
    known = known_adapters()
    print('connected adapters:')
    for device in serial_devices():
        serial = properties(device)['ID_SERIAL_SHORT']
        print('  {:<14} serial {:<14} {:<22} {}'.format(
            device, serial, '/dev/' + by_serial[serial] if serial in by_serial else '(no name)',
            'listed as ' + known[serial] if serial in known else 'not in ' + os.path.basename(ADAPTERS)))
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


def from_list(devices):
    """[(device, role)] for the connected adapters without a name, from usb_adapters.yaml."""
    named = {serial for _, serial in installed_rules().values()}
    known = known_adapters()
    roles = []
    unknown = []
    for device in devices:
        serial = properties(device)['ID_SERIAL_SHORT']
        if serial in named:
            print('{} (serial {}): already named, kept'.format(device, serial))
        elif serial in known:
            print('{} (serial {}): listed as the {}'.format(device, serial, known[serial]))
            roles.append((device, known[serial]))
        else:
            unknown.append('{} (serial {})'.format(device, serial))
    if unknown:
        sys.exit('{} not in {}: say which arm it drives with --role leader|follower '
                 '(and --device if several are connected)'.format(', '.join(unknown), ADAPTERS))
    return roles


def main():
    args = parse_args()
    if args.list:
        show_list()
        return
    new_adapters = []
    if args.role is not None:
        device = args.device or find_device()
        serial = properties(device)['ID_SERIAL_SHORT']
        listed = known_adapters().get(serial)
        if listed is not None and listed != args.role:
            sys.exit('{} (serial {}) is listed as the {} in {}; fix that file if it changed'.format(
                device, serial, listed, ADAPTERS))
        if listed is None:
            new_adapters.append((serial, args.role))
        targets = [(device, args.role)]
    else:
        devices = [args.device] if args.device else serial_devices()
        if not devices:
            sys.exit('no /dev/ttyACM* or /dev/ttyUSB* found: plug the adapter in')
        targets = from_list(devices)
        if not targets:
            print('nothing to do: every connected adapter already has a name (see --list)')
            return
    rules = [(name, plan_rule(device, role, name, args.replace))
             for device, role in targets
             for name in [args.name or NAMES[role]]]
    if not args.apply:
        for serial, role in new_adapters:
            print('--apply also adds "{}: {}" to {}'.format(serial, role, os.path.normpath(ADAPTERS)))
        print('dry run; run with sudo and --apply to install')
        return
    if os.geteuid() != 0:
        sys.exit('--apply writes {}: run with sudo'.format(RULE_DIR))
    for _, (path, rule) in rules:
        with open(path, 'w') as f:
            f.write(rule)
    for serial, role in new_adapters:
        remember_adapter(serial, role)
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
