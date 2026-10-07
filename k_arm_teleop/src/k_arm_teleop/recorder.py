"""``rosbag record`` as a child process, shared by teleop_manager.py and record_only.py."""

import datetime
import glob
import os
import shutil
import signal
import subprocess

import rospy


class Recorder(object):
    """A ``rosbag record`` child process plus statistics of its directory."""

    def __init__(self, directory, prefix, topics):
        self.directory = os.path.expanduser(directory)
        self.prefix = prefix
        self.topics = topics
        self.process = None
        self.bag_path = None
        self.started = None

    def set_directory(self, directory):
        if self.process is not None:
            raise RuntimeError('stop recording before changing the directory')
        directory = os.path.abspath(os.path.expanduser(directory))
        if not os.path.isdir(directory):
            os.makedirs(directory)
        self.directory = directory

    def start(self):
        if self.process is not None:
            raise RuntimeError('already recording')
        if not os.path.isdir(self.directory):
            os.makedirs(self.directory)
        stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
        self.bag_path = os.path.join(self.directory, '{}_{}.bag'.format(self.prefix, stamp))
        command = ['rosbag', 'record', '-O', self.bag_path,
                   '__name:=k_arm_teleop_recorder'] + list(self.topics)
        # own process group, so SIGINT reaches rosbag's children too
        self.process = subprocess.Popen(command, preexec_fn=os.setsid)
        self.started = rospy.get_time()

    def stop(self):
        if self.process is None:
            raise RuntimeError('not recording')
        os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
        try:
            self.process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            self.process.wait()
        path = self.bag_path
        self.process = None
        self.started = None
        return path

    def poll(self):
        """Return an error message if the child died on its own, else None."""
        if self.process is None or self.process.poll() is None:
            return None
        code = self.process.returncode
        self.process = None
        self.started = None
        return 'rosbag record exited with code {}'.format(code)

    def current_size(self):
        for path in (self.bag_path + '.active', self.bag_path):
            if self.bag_path and os.path.exists(path):
                return os.path.getsize(path)
        return 0

    def directory_stats(self):
        if not os.path.isdir(self.directory):
            return 0, 0, 0
        bags = glob.glob(os.path.join(self.directory, '*.bag')) + \
            glob.glob(os.path.join(self.directory, '*.bag.active'))
        total = sum(os.path.getsize(b) for b in bags if os.path.exists(b))
        finished = len([b for b in bags if b.endswith('.bag')])
        return finished, total, shutil.disk_usage(self.directory).free
