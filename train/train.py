#!/usr/bin/env python

import argparse
import datetime
import os.path as osp
import subprocess
import sys

import six


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
    parser.add_argument("--ip", type=str, default='133.11.216.13', help="IP Address")
    parser.add_argument(
        "-i", '--identity-file', type=str,
        default=osp.join(osp.expanduser('~'), '.ssh', 'id_rsa'), help="SSH Identify File")
    parser.add_argument("image_directory", type=str, help="Image Directory")
    args = parser.parse_args()
    print(args.identity_file)

    proxy_command = "ssh -i {} -o ProxyCommand='ssh -i {} -W %h:%p {}@dlbox2.jsk.imi.i.u-tokyo.ac.jp'".format(
        args.identity_file, args.identity_file, args.username)
    ssh_target = '{}@{}'.format(args.username, args.ip)
    ssh_command = "{} -i {} {}@{}".format(proxy_command, args.identity_file,
                                          args.username, args.ip)

    tmp_dir = osp.join('/tmp', 'project-t', '{}'.format(current_time_str()))
    a = run_command('{} mkdir -p {}'.format(ssh_command, tmp_dir), shell=True)

    source_image_dir = args.image_directory.rstrip('/')
    rsync_image_command = 'rsync -e "{}" --verbose -r {} {}:{}'.format(
        proxy_command,
        source_image_dir, ssh_target, tmp_dir)
    run_command(rsync_image_command, shell=True)

    source_image_dir_in_remote = osp.join(tmp_dir, osp.basename(source_image_dir))

    n_proc = run_command(
        '''{} 'bash --login -c "check.sh"' '''.format(
            ssh_command), shell=True, capture_output=True).stdout
    if int(n_proc) > 0:
        print(Colors.red + "Can't run it now because another training process is already running. Please wait for a while and execute." + Colors.reset)
        sys.exit(1)

    try:
        run_command(
            '''{} 'bash --login -c "run.sh {}"' '''.format(
                ssh_command, source_image_dir_in_remote),
            shell=True)
    except KeyboardInterrupt:
        print(Colors.red + "[KeyboardInterrupt] Stop training script." + Colors.reset)
        run_command(
            '''{} 'bash --login -c "kill.sh"' '''.format(
                ssh_command), shell=True)
        sys.exit(1)

    date = current_time_str()
    saved_weight_name = '{}-{}.pt'.format(osp.basename(source_image_dir), date)
    rsync_image_command = 'rsync -e "{}" --verbose {}:{} ./{}'.format(
        proxy_command,
        ssh_target, '{}/generated_data/yolov7-seg-coco/weights/best.pt'.format(source_image_dir_in_remote),
        saved_weight_name)
    run_command(rsync_image_command, shell=True)

    saved_yaml_name = '{}-{}.yaml'.format(osp.basename(source_image_dir), date)
    rsync_image_command = 'rsync -e "{}" --verbose {}:{} ./{}'.format(
        proxy_command,
        ssh_target, '{}/generated_data/from_images_dir.yaml'.format(source_image_dir_in_remote),
        saved_yaml_name)
    run_command(rsync_image_command, shell=True)

    saved_rembg_dir_name = '{}-{}-preprocessing'.format(osp.basename(source_image_dir), date)
    rsync_image_command = 'rsync -r -e "{}" --verbose {}:{} ./{}'.format(
        proxy_command,
        ssh_target, '{}/generated_data/preprocessing'.format(source_image_dir_in_remote),
        saved_rembg_dir_name)
    run_command(rsync_image_command, shell=True)

    saved_annotation_filename = '{}-{}-generated_data.tar.gz'.format(osp.basename(source_image_dir), date)
    rsync_image_command = 'rsync -r -e "{}" --verbose {}:{} ./{}'.format(
        proxy_command,
        ssh_target, '{}/generated_data/generated_data.tar.gz'.format(source_image_dir_in_remote),
        saved_annotation_filename)
    run_command(rsync_image_command, shell=True)
    print(Colors.green + "Done copying model file for pytorch object detection" + Colors.reset)
    print(Colors.green + " - {}".format(saved_weight_name) + Colors.reset)
    print(Colors.green + " - {}".format(saved_yaml_name) + Colors.reset)
    print(Colors.green + " - {}".format(saved_rembg_dir_name) + Colors.reset)
    print(Colors.green + " - {}".format(saved_annotation_filename) + Colors.reset)
