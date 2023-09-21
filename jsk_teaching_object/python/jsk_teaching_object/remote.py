import datetime
import os
import os.path as osp
from pathlib import Path
import subprocess
import sys
import tempfile
import time

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


def run_ssh_task(
        image_directory,
        username="iory", bastion_username="iory", bastion_ip="dlbox2.jsk.imi.i.u-tokyo.ac.jp",
        output_ip="", output_username="", ip='133.11.216.103', epoch=1,
        identity_file=osp.join(osp.expanduser('~'), '.ssh', 'id_rsa'), output='',
        batchsize=16,
):
    proxy_command = "ssh -i {} -o ProxyCommand='ssh -i {} -W %h:%p {}@{}'".format(
        identity_file, identity_file, bastion_username, bastion_ip)
    ssh_target = '{}@{}'.format(username, ip)
    ssh_command = "{} -i {} {}@{}".format(proxy_command, identity_file, username, ip)

    output_to_remote = False
    if len(output_ip) > 0 and len(output_username) > 0:
        tmp_output = tempfile.TemporaryDirectory()
        makedirs(osp.join(tmp_output.name, osp.dirname(output)))
        output_to_remote = True

    if output_to_remote is False and len(output) > 0:
        output = output.rstrip('/')
        makedirs(osp.dirname(output))

    project_id = current_time_str()
    tmp_dir = osp.join('/tmp', 'project-t', '{}'.format(project_id))
    print('tmp_dir = {}'.format(tmp_dir))
    a = run_command('{} mkdir -p {}'.format(ssh_command, tmp_dir), shell=True)

    source_image_dir = image_directory.rstrip('/')
    rsync_image_command = 'rsync -e "{}" --verbose -r {} {}:{}'.format(
        proxy_command, source_image_dir, ssh_target, tmp_dir)
    run_command(rsync_image_command, shell=True)
    source_image_dir_in_remote = osp.join(tmp_dir, osp.basename(source_image_dir))

    session_name = project_id
    print(ssh_command + ' ' + ' '.join(["tmux", "new-session", "-d", "-s", session_name]))
    run_command(ssh_command + ' ' + ' '.join(["tmux", "new-session", "-d", "-s", session_name]), shell=True)
    command = f'cd /home/iory/src/github.com/iory/jsk_demos/train && python generate_data.py --from-images-dir {source_image_dir_in_remote} -b {batchsize} --epoch {epoch} --compress-annotation-data'
    print(ssh_command + ' ' + ' '.join(["tmux", "send-keys", "-t", session_name, "'{}'".format(command), "Enter"]))
    run_command(ssh_command + ' ' + '"' + ' '.join(["tmux", "send-keys", "-t", session_name, "'{}'".format(command), "Enter"]) + '"', shell=True)
    return source_image_dir_in_remote, session_name


def watchdog(bastion_username, bastion_ip, username, ip, remote_file_path, identity_filepath=None, poll_interval=5):
    if identity_filepath is None:
        identity_filepath = osp.join(osp.expanduser('~'), '.ssh', 'id_rsa')

    proxy_command = "ssh -i {} -o ProxyCommand='ssh -i {} -W %h:%p {}@{}'".format(
        identity_filepath, identity_filepath, bastion_username, bastion_ip)

    ssh_command = "{} -i {} {}@{}".format(proxy_command, identity_filepath, username, ip)

    check_file_cmd = f"{ssh_command} test -f {remote_file_path} && echo exists || echo not_exists"

    while True:
        result = run_command(check_file_cmd, shell=True, capture_output=True)
        if "exists" == result.stdout.decode('utf-8').strip():
            break
        else:
            print(f'waiting for the file {remote_file_path} to be created on the remote server.')
        time.sleep(poll_interval)



def kill_tmux_session(
        session_name, remote_file_path,
        username="iory", bastion_username="iory", bastion_ip="dlbox2.jsk.imi.i.u-tokyo.ac.jp",
        output_ip="", output_username="", ip='133.11.216.103',
        identity_file=osp.join(osp.expanduser('~'), '.ssh', 'id_rsa'),
):
    proxy_command = "ssh -i {} -o ProxyCommand='ssh -i {} -W %h:%p {}@{}'".format(
        identity_file, identity_file, bastion_username, bastion_ip)
    '{}@{}'.format(username, ip)
    ssh_command = "{} -i {} {}@{}".format(proxy_command, identity_file, username, ip)

    run_command(ssh_command + ' ' + ' '.join(["tmux", "kill-session", '-t', session_name]), shell=True)


def train_in_remote(
        image_directory='/home/iory/src/github.com/jsk-ros-pkg/jsk_demos/train/yamagata_items',
        bastion_username = 'iory',
        bastion_ip = 'dlbox2.jsk.imi.i.u-tokyo.ac.jp',
        username = 'iory',
        ip = '133.11.216.103',
        output='',
        output_ip='',
        output_username='',
        identity_file=osp.join(osp.expanduser('~'), '.ssh', 'id_rsa')):
    source_image_dir_in_remote, session_name = run_ssh_task(image_directory)
    remote_file_path = Path(source_image_dir_in_remote) / 'generated_data' / 'yolov7-seg-coco' / 'weights' / 'best.pt'

    watchdog(bastion_username, bastion_ip, username, ip, remote_file_path,
             identity_filepath=identity_file)
    kill_tmux_session(session_name, remote_file_path, identity_file=identity_file)

    output_to_remote = False
    if len(output_ip) > 0 and len(output_username) > 0:
        tmp_output = tempfile.TemporaryDirectory()
        makedirs(osp.join(tmp_output.name, osp.dirname(output)))
        output_to_remote_filename_pairs = []
        output_to_remote = True

    if output_to_remote is False and len(output) > 0:
        output = output.rstrip('/')
        makedirs(osp.dirname(output))

    if len(output) > 0:
        if output_to_remote:
            saved_weight_name = osp.join(tmp_output.name, '{}.pt'.format(output))
            output_to_remote_filename_pairs.append(
                (saved_weight_name, '{}.pt'.format(output)))
        else:
            saved_weight_name = '{}.pt'.format(output)
    else:
        saved_weight_name = './{}-{}.pt'.format(osp.basename(source_image_dir), date)

    ssh_target = '{}@{}'.format(username, ip)
    proxy_command = "ssh -i {} -o ProxyCommand='ssh -i {} -W %h:%p {}@{}'".format(
        identity_file, identity_file, bastion_username, bastion_ip)
    rsync_image_command = 'rsync -e "{}" --verbose {}:{} {}'.format(
        proxy_command,
        ssh_target, '{}/generated_data/yolov7-seg-coco/weights/best.pt'.format(source_image_dir_in_remote),
        saved_weight_name)
    run_command(rsync_image_command, shell=True)

    if len(output) > 0:
        if output_to_remote:
            saved_yaml_name = osp.join(tmp_output.name, '{}.yaml'.format(output))
            output_to_remote_filename_pairs.append(
                (saved_yaml_name, '{}.yaml'.format(output)))
        else:
            saved_yaml_name = '{}.yaml'.format(output)
    else:
        saved_yaml_name = './{}-{}.yaml'.format(osp.basename(source_image_dir), date)
    rsync_image_command = 'rsync -e "{}" --verbose {}:{} {}'.format(
        proxy_command,
        ssh_target, '{}/generated_data/from_images_dir.yaml'.format(source_image_dir_in_remote),
        saved_yaml_name)
    run_command(rsync_image_command, shell=True)

    return saved_weight_name, saved_yaml_name


if __name__ == '__main__':
    from eos import measure
    with measure():
        filename = '{}.pt'.format(current_time_str())
        saved_weight_filepath, saved_yaml_name = train_in_remote(
            image_directory='/home/iory/src/github.com/jsk-ros-pkg/jsk_demos/train/tiny_yamagata_items',
            output=osp.join(osp.expanduser('~'), 'dataset', '2023-09-21', filename))
        print(saved_weight_filepath, saved_yaml_name)
