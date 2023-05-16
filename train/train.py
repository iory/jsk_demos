#!/usr/bin/env python

import argparse
import os.path as osp
import subprocess
import datetime


def current_time_str(time_format='%Y-%m-%d-%H-%M-%S-%f'):
    time_str = datetime.datetime.now().strftime(time_format)
    return time_str


def run_command(cmd, *args, **kwargs):
    if kwargs.pop("capture_output", False):
        kwargs["stdout"] = subprocess.PIPE
        kwargs["stderr"] = subprocess.PIPE
    return subprocess.run(cmd, *args, **kwargs)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Image training Script")
    parser.add_argument('-u', '--username', type=str, default="thk")
    parser.add_argument("--ip", type=str, default='133.11.216.13', help="IP Address")
    parser.add_argument("image_directory", type=str, help="Image Directory")
    args = parser.parse_args()

    proxy_command = "ssh -o ProxyCommand='ssh -W %h:%p {}@dlbox2.jsk.imi.i.u-tokyo.ac.jp'".format(args.username)
    ssh_target = '{}@{}'.format(args.username, args.ip)
    ssh_command = "{} {}@{}".format(proxy_command, args.username, args.ip)

    tmp_dir = osp.join('/tmp', 'project-t', '{}'.format(current_time_str()))
    a = run_command('{} mkdir -p {}'.format(ssh_command, tmp_dir), shell=True)

    source_image_dir = args.image_directory.rstrip('/')
    rsync_image_command = 'rsync -e "{}" --verbose -r {} {}:{}'.format(
        proxy_command,
        source_image_dir, ssh_target, tmp_dir)
    print(rsync_image_command)
    run_command(rsync_image_command, shell=True)

    source_image_dir_in_remote = osp.join(tmp_dir, osp.basename(source_image_dir))

    python_exe = 'python'
    run_command(
        '''{} 'bash --login -c "{} generate_data.py --from-images-dir {} -b 32"' '''.format(
            ssh_command, python_exe, source_image_dir_in_remote),
        shell=True)

    date = current_time_str()
    rsync_image_command = 'rsync -e "{}" --verbose {}:{} ./{}'.format(
        proxy_command,
        ssh_target, '/tmp/from_images_dir/yolov7-seg-coco/weights/best.pt',
        'best-{}.pt'.format(date))
    run_command(rsync_image_command, shell=True)

    rsync_image_command = 'rsync -e "{}" --verbose {}:{} ./{}'.format(
        proxy_command,
        ssh_target, '/tmp/from_images_dir/from_images_dir.yaml',
        'from_images_dir-{}.yaml'.format(date))
    run_command(rsync_image_command, shell=True)
