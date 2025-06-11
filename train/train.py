#!/usr/bin/env python

import argparse
import datetime
import os
import os.path as osp
import subprocess
import sys
import tempfile

import six


PY3 = (sys.version_info[0] == 3)
PY2 = not PY3


def makedirs(name, mode=0o777, exist_ok=True):
    """An wrapper of os.makedirs that accepts exist_ok.

    Parameters
    ----------
    name : str
        path of directory
    exist_ok : bool
        if True, accepts the existence of the directory.

    Examples
    --------
    >>> from eos import makedirs
    >>> makedirs('/tmp/result_directory')
    """
    name = str(name)
    if PY2:
        try:
            os.makedirs(name, mode)
        except OSError:
            if not (exist_ok and os.path.isdir(name)):
                raise OSError(
                    'Directory {} already exists. '
                    'Set exist_ok = True if the directory can exist.'
                    .format(name))
    else:
        os.makedirs(name, mode, exist_ok=exist_ok)


class Colors(object):

    bold = '\033[1m'
    underlined = '\033[4m'

    black = '\033[30m'
    red = '\033[31m'
    green = '\033[32m'
    yellow = '\033[33m'
    blue = '\033[34m'
    magenta = '\033[35m'
    cyan = '\033[36m'
    lightgray = '\033[37m'
    darkgray = '\033[90m'
    lightred = '\033[91m'
    lightgreen = '\033[92m'
    lightyellow = '\033[93m'
    lightblue = '\033[94m'
    lightmagenta = '\033[95m'
    lightcyan = '\033[96m'

    background_black = '\033[40m'
    background_red = '\033[41m'
    background_green = '\033[42m'
    background_yellow = '\033[43m'
    background_blue = '\033[44m'
    background_magenta = '\033[45m'
    background_cyan = '\033[46m'

    reset = '\033[0m'


def current_time_str(time_format='%Y-%m-%d-%H-%M-%S-%f'):
    time_str = datetime.datetime.now().strftime(time_format)
    return time_str


def run_command(cmd, *args, **kwargs):
    if kwargs.pop("capture_output", False):
        kwargs["stdout"] = subprocess.PIPE
        kwargs["stderr"] = subprocess.PIPE
    if six.PY2:
        return subprocess.check_call(cmd, *args, **kwargs)
    else:
        return subprocess.run(cmd, *args, **kwargs)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Image training Script")
    parser.add_argument('-u', '--username', type=str, default="thk")
    parser.add_argument('-bu', '--bastion-username', type=str, default=None)
    parser.add_argument('-bip', '--bastion-ip', type=str, default=None)
    parser.add_argument('-oip', '--output-ip', type=str, default="",
                        help='Specifies the IP address of the remote PC. '
                        'This option is required if you wish to transfer the file.')
    parser.add_argument('-ou', '--output-username', type=str, default="",
                        help='Specifies the username for the remote PC. '
                        'This option is required if you wish to transfer the file.')
    parser.add_argument("--ip", type=str, default='133.11.216.222', help="IP Address")
    parser.add_argument('-e', "--epoch", type=int, default=10, help="Training epoch")
    parser.add_argument(
        "-i", '--identity-file', type=str,
        default=osp.join(osp.expanduser('~'), '.ssh', 'id_rsa'), help="SSH Identify File")
    parser.add_argument(
        "-o", '--output', type=str,
        default='', help="Output prefix filename.")
    parser.add_argument("image_directory", type=str, help="Image Directory")
    args = parser.parse_args()

    proxy_command = ""
    ssh_command = ""
    if args.bastion_ip and args.bastion_username:
        proxy_command = "ssh -i {} -o ProxyCommand='ssh -i {} -W %h:%p {}@{}'".format(
            args.identity_file, args.identity_file,
            args.bastion_username, args.bastion_ip)
        ssh_command = "{} -i {} {}@{}".format(
            proxy_command, args.identity_file,
            args.username, args.ip)
    else:
        ssh_command = "ssh -i {} {}@{}".format(args.identity_file, args.username, args.ip)

    output_to_remote = False
    if len(args.output_ip) > 0 and len(args.output_username) > 0:
        tmp_output = tempfile.TemporaryDirectory()
        makedirs(osp.join(tmp_output.name, osp.dirname(args.output)))
        output_to_remote_filename_pairs = []
        output_to_remote = True

    if output_to_remote is False and len(args.output) > 0:
        args.output = args.output.rstrip('/')
        makedirs(osp.dirname(args.output))

    n_proc = run_command(
        '''{} 'bash --login -c "check.sh"' '''.format(
            ssh_command), shell=True, capture_output=True).stdout
    if len(n_proc) == 0:
        print(Colors.red + "Could you check the following command" + Colors.reset)
        print('''{} 'bash --login -c "check.sh"' '''.format(ssh_command))
        sys.exit(1)
    if int(n_proc) > 0:
        print(Colors.red + "Can't run it now because another training process is already running. Please wait for a while and execute." + Colors.reset)
        sys.exit(1)

    tmp_dir = osp.join('/tmp', 'project-t', '{}'.format(current_time_str()))
    run_command('{} mkdir -p {}'.format(ssh_command, tmp_dir), shell=True)

    source_image_dir = args.image_directory.rstrip('/')
    rsync_image_command = 'rsync -e "{}" --verbose -r {} {}:{}'.format(
        proxy_command if proxy_command else "ssh -o ServerAliveInterval=60 -i {}".format(args.identity_file),
        source_image_dir, ssh_command.split(' ')[-1], tmp_dir)
    run_command(rsync_image_command, shell=True)

    source_image_dir_in_remote = osp.join(tmp_dir, osp.basename(source_image_dir))
    n_proc = run_command(
        '''{} 'bash --login -c "check.sh"' '''.format(
            ssh_command), shell=True, capture_output=True).stdout
    if int(n_proc) > 0:
        print(Colors.red + "Can't run it now because another training process is already running. Please wait for a while and execute." + Colors.reset)
        sys.exit(1)

    try:
        print('''{} 'bash --login -c "run.sh {} -e {} -o /workspace/target_data/gen_data"' '''.format(
                ssh_command, source_image_dir_in_remote, args.epoch))
        run_command(
            '''{} 'bash --login -c "run.sh {} -e {} -o /workspace/target_data/gen_data"' '''.format(
                ssh_command, source_image_dir_in_remote, args.epoch),
            shell=True)
    except KeyboardInterrupt:
        print(Colors.red + "[KeyboardInterrupt] Stop training script." + Colors.reset)
        run_command(
            '''{} 'bash --login -c "kill.sh"' '''.format(
                ssh_command), shell=True)
        sys.exit(1)

    date = current_time_str()
    if len(args.output) > 0:
        if output_to_remote:
            saved_weight_name = osp.join(tmp_output.name, '{}.pt'.format(args.output))
            output_to_remote_filename_pairs.append(
                (saved_weight_name, '{}.pt'.format(args.output)))
        else:
            saved_weight_name = '{}.pt'.format(args.output)
    else:
        makedirs('./{}-generated_data'.format(osp.basename(source_image_dir)))
        saved_weight_name = './{}-generated_data/{}.pt'.format(osp.basename(source_image_dir),
                                                               osp.basename(source_image_dir))
    rsync_image_command = 'rsync -e "{}" --verbose {}:{} {}'.format(
        proxy_command if proxy_command else "ssh -o ServerAliveInterval=60 -i {}".format(args.identity_file),
        ssh_command.split(' ')[-1], '{}/gen_data/train/weights/best.pt'.format(source_image_dir_in_remote),
        saved_weight_name)
    print(rsync_image_command)
    run_command(rsync_image_command, shell=True)

    if len(args.output) > 0:
        if output_to_remote:
            saved_yaml_name = osp.join(tmp_output.name, '{}.yaml'.format(args.output))
            output_to_remote_filename_pairs.append(
                (saved_yaml_name, '{}.yaml'.format(args.output)))
        else:
            saved_yaml_name = '{}.yaml'.format(args.output)
    else:
        saved_yaml_name = './{}-generated_data/{}.yaml'.format(
            osp.basename(source_image_dir),
            osp.basename(source_image_dir))
    rsync_image_command = 'rsync -e "{}" --verbose {}:{} {}'.format(
        proxy_command if proxy_command else "ssh -o ServerAliveInterval=60 -i {}".format(args.identity_file),
        ssh_command.split(' ')[-1], '{}/gen_data/config.yaml'.format(source_image_dir_in_remote),
        saved_yaml_name)
    run_command(rsync_image_command, shell=True)

    if len(args.output) > 0:
        if output_to_remote:
            saved_rembg_dir_name = osp.join(tmp_output.name, '{}'.format(args.output))
        else:
            saved_rembg_dir_name = '{}'.format(args.output)
    else:
        saved_rembg_dir_name = './{}-generated_data/{}-preprocessing'.format(osp.basename(source_image_dir),
                                                                             osp.basename(source_image_dir))
    rsync_image_command = 'rsync -r -e "{}" --verbose {}:{} {}'.format(
        proxy_command if proxy_command else "ssh -o ServerAliveInterval=60 -i {}".format(args.identity_file),
        ssh_command.split(' ')[-1], '{}/gen_data/preprocessing'.format(source_image_dir_in_remote),
        saved_rembg_dir_name)
    run_command(rsync_image_command, shell=True)

    print(Colors.green + "Done copying model file for pytorch object detection" + Colors.reset)
    print(Colors.green + " - {}".format(saved_weight_name) + Colors.reset)
    print(Colors.green + " - {}".format(saved_yaml_name) + Colors.reset)
    print(Colors.green + " - {}".format(saved_rembg_dir_name) + Colors.reset)

    if len(args.output_ip) > 0 and len(args.output_username) > 0:
        out_ssh_target = "{}@{}".format(args.output_username, args.output_ip)

        output_ssh_command = "ssh {}@{} 'mkdir -p {}'".format(
            args.output_username, args.output_ip, osp.dirname(args.output.rstrip('/')))
        run_command(output_ssh_command, shell=True)
        dsts = []
        for frm, dst in output_to_remote_filename_pairs:
            rsync_file_command = 'rsync --verbose {} {}:{}'.format(
                frm, out_ssh_target, dst)
            run_command(rsync_file_command, shell=True)
            dsts.append(dst)
        print(Colors.green + "Done copying model file for pytorch object detection to remote PC" + Colors.reset)
        for dst in dsts:
            print(Colors.green + " - {}".format(dst) + Colors.reset)
