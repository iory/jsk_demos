#!/usr/bin/env python3
"""Train and run the learned picking policy from the rviz panel.

Wraps scripts/kpick.sh: ``~train`` runs ``kpick.sh train`` on a directory of
bags (the server fine-tunes, the result comes back to ``~results_dir``);
``~launch`` starts launch/kpick_right.launch with a result, in preview
(nothing goes to the robot) or execute mode, optionally in the node's step
mode (it shows the next steps and runs them on its ``~step``);
``~shutdown`` stops it.
Starting and stopping an episode is the policy node's own ``~start`` /
``~stop``. ``~status`` (PolicyStatus) reports both.

Execute is refused while the teleop drives the real robot; the policy node
also refuses to start while the teleop publishes commands.

Parameters
----------
~results_dir : str
    Where kpick_train.sh leaves its results (default ~/kpick_results).
~sim_trees : int
    Trees of the server's simulation check per training (kpick_train.sh -e,
    default 20).
~teleop_status : str
    The teleop manager's status topic.
~policy_status : str
    The policy node's status topic (std_msgs/String JSON), if it has one.
"""

import glob
import json
import os
import re
import signal
import subprocess
import threading

import rospkg
import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse

from k_arm_teleop.msg import PolicyStatus
from k_arm_teleop.msg import TeleopStatus
from k_arm_teleop.srv import SetString
from k_arm_teleop.srv import SetStringResponse

ACTIVE_TELEOP = ('approaching', 'following', 'moving_to_zero')
JOB_LINE = re.compile(r'\[kpick\] job (\S+):')


def describe_policy_status(data):
    """One line from the policy node's JSON status; the raw text if it is not JSON."""
    try:
        st = json.loads(data)
    except ValueError:
        return data
    parts = [st.get('state', '?')]
    if st.get('state') != 'idle':
        parts[0] += ' {}/{}'.format(st.get('step', '?'), st.get('max_steps', '?'))
    if st.get('stop_reason') and st.get('state') == 'stopped':
        parts[0] += ' ({})'.format(st['stop_reason'])
    target = st.get('target') or {}
    if target.get('detected'):
        parts.append('apple at {} ({} seen)'.format(
            ', '.join('{:+.2f}'.format(v) for v in target.get('position', [])), target.get('n_apples', '?')))
    else:
        parts.append('no apple detected')
    prediction = st.get('prediction') or {}
    if st.get('state') == 'awaiting_step' and prediction:
        parts.append('next {} steps: {:.0f} mm{}'.format(
            prediction.get('n_steps', '?'), prediction.get('move_mm', 0),
            ', gripper closes' if prediction.get('gripper_closes') else ''))
    if st.get('teleop_active'):
        parts.append('teleop is commanding')
    tick = st.get('tick_ms') or {}
    if 'median' in tick:
        parts.append('tick {:.0f} ms (max {:.0f})'.format(tick['median'], tick.get('max', 0)))
    return ' / '.join(parts)


class KpickManager(object):

    def __init__(self):
        self.results_dir = os.path.expanduser(rospy.get_param('~results_dir', '~/kpick_results'))
        self.sim_trees = int(rospy.get_param('~sim_trees', 20))
        self.policy_node = rospy.get_param('~policy_node', '/kpick_policy')
        self.script = os.path.join(rospkg.RosPack().get_path('k_arm_teleop'), 'scripts', 'kpick.sh')
        self.lock = threading.Lock()
        self.train_proc = None
        self.train_job = ''
        self.train_stage = ''
        self.train_error = ''
        self.policy_proc = None
        self.policy_mode = ''
        self.policy_result = ''
        self.policy_status = ''
        self.policy_status_time = None
        self.policy_state = ''
        self.step_mode = False
        self.teleop = None
        self.message = 'ready'
        rospy.Subscriber(rospy.get_param('~teleop_status', '/k_arm_teleop_manager/status'),
                         TeleopStatus, self._teleop_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param('~policy_status', '/kpick_policy/status'),
                         String, self._policy_status_cb, queue_size=1)
        self.status_pub = rospy.Publisher('~status', PolicyStatus, queue_size=1)
        rospy.Service('~train', SetString, self._train)
        rospy.Service('~cancel_train', Trigger, self._cancel_train)
        rospy.Service('~launch', SetString, self._launch)
        rospy.Service('~shutdown', Trigger, self._shutdown_policy)
        rospy.on_shutdown(self._on_shutdown)

    # -- inputs -------------------------------------------------------------------

    def _teleop_cb(self, msg):
        self.teleop = msg

    def _policy_status_cb(self, msg):
        text = describe_policy_status(msg.data)
        try:
            st = json.loads(msg.data)
        except ValueError:
            st = {}
        with self.lock:
            self.policy_status = text
            self.policy_state = st.get('state', '')
            self.step_mode = bool(st.get('step_mode', False))
            self.policy_status_time = rospy.get_time()

    def results(self):
        found = glob.glob(os.path.join(self.results_dir, '*', 'result'))
        return sorted(found, key=os.path.getmtime, reverse=True)

    # -- training -----------------------------------------------------------------

    def _train(self, request):
        directory = os.path.expanduser(request.data.strip())
        if not directory and self.teleop is not None:
            directory = os.path.expanduser(self.teleop.record_directory)
        with self.lock:
            if self.train_proc is not None:
                return SetStringResponse(False, 'training is already running ({})'.format(
                    self.train_job or 'starting'))
            bags = glob.glob(os.path.join(directory, '*.bag'))
            if not bags:
                return SetStringResponse(False, 'no .bag in {}'.format(directory or '(no directory)'))
            if self.teleop is not None and self.teleop.recording:
                return SetStringResponse(False, 'stop recording first: the bag being written '
                                                'would be sent half-finished')
            os.makedirs(self.results_dir, exist_ok=True)
            command = [self.script, 'train', '-e', str(self.sim_trees), directory]
            self.train_proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                               universal_newlines=True, bufsize=1,
                                               start_new_session=True)
            self.train_job = ''
            self.train_stage = 'sending {} bags from {}'.format(len(bags), directory)
            self.train_error = ''
        threading.Thread(target=self._watch_training, args=(self.train_proc,), daemon=True).start()
        rospy.loginfo('training: %s', ' '.join(command))
        return SetStringResponse(True, 'training on {} bags in {}'.format(len(bags), directory))

    def _watch_training(self, proc):
        log_path = os.path.join(self.results_dir, 'panel_train.log')
        last_lines = []
        with open(log_path, 'a') as log:
            for line in proc.stdout:
                log.write(line)
                log.flush()
                line = line.rstrip()
                last_lines = (last_lines + [line])[-5:]
                with self.lock:
                    match = JOB_LINE.search(line)
                    if match:
                        self.train_job = match.group(1)
                    if line.startswith('[kpick'):
                        self.train_stage = line
        code = proc.wait()
        with self.lock:
            self.train_proc = None
            if code == 0:
                self.train_stage = 'done: {}'.format(self.results()[0] if self.results() else '?')
                self.message = 'training finished'
            else:
                failed = [l for l in last_lines if 'FAILED' in l]
                self.train_error = (failed or last_lines or ['exit code {}'.format(code)])[-1]
                self.message = 'training failed (log: {})'.format(log_path)
        rospy.loginfo('training ended with %d', code)

    def _cancel_train(self, _request):
        with self.lock:
            proc = self.train_proc
        if proc is None:
            return TriggerResponse(False, 'no training is running')
        os.killpg(proc.pid, signal.SIGINT)
        return TriggerResponse(True, 'stopped waiting for {}; a job already on the server keeps running '
                                     'there'.format(self.train_job or 'the job'))

    # -- running ------------------------------------------------------------------

    def _launch(self, request):
        words = request.data.split()
        step = 'step' in words[1:]
        words = [w for w in words if w != 'step']
        if not words or words[0] not in ('preview', 'execute'):
            return SetStringResponse(False, 'expected "preview|execute [result_dir] [step]"')
        mode = words[0]
        result = os.path.expanduser(words[1]) if len(words) > 1 else (self.results() or [''])[0]
        if not os.path.isdir(result):
            return SetStringResponse(False, 'no result directory {}'.format(result or '(none yet)'))
        if mode == 'execute' and self.teleop is not None and self.teleop.target == 'real' \
                and self.teleop.phase in ACTIVE_TELEOP:
            return SetStringResponse(False, 'the teleop drives the real robot ({}): press STOP first'.format(
                self.teleop.phase))
        with self.lock:
            if self.policy_proc is not None:
                return SetStringResponse(False, 'a policy is already running ({}); shut it down first'.format(
                    self.policy_mode))
            command = ['roslaunch', 'k_arm_teleop', 'kpick_right.launch', 'result:=' + result,
                       'execute:=' + ('true' if mode == 'execute' else 'false')]
            self.policy_proc = subprocess.Popen(command, start_new_session=True)
            self.policy_mode = mode
            self.policy_result = result
            self.policy_status = ''
            self.message = 'policy {} starting'.format(mode)
        threading.Thread(target=self._watch_policy, args=(self.policy_proc,), daemon=True).start()
        if step:
            threading.Thread(target=self._enable_step_mode, args=(self.policy_proc,), daemon=True).start()
        rospy.loginfo('policy: %s', ' '.join(command))
        return SetStringResponse(True, '{}{}: {}'.format(mode, ' (step mode)' if step else '', result))

    def _enable_step_mode(self, proc):
        """Switch the freshly launched node to step mode once its service is up."""
        service = self.policy_node + '/set_step_mode'
        deadline = rospy.get_time() + 120.0  # the node loads the detector and the policy first
        while not rospy.is_shutdown() and proc.poll() is None and rospy.get_time() < deadline:
            try:
                rospy.wait_for_service(service, timeout=1.0)
            except rospy.ROSException:
                continue
            try:
                response = rospy.ServiceProxy(service, SetBool)(True)
            except rospy.ServiceException as error:
                response = None
                rospy.logerr('%s: %s', service, error)
            with self.lock:
                self.message = 'step mode on' if response is not None and response.success else \
                    'could not switch the policy to step mode: press Shut down'
            return
        if proc.poll() is None:
            with self.lock:
                self.message = 'no {} after 120 s: is this result older than step mode?'.format(service)

    def _watch_policy(self, proc):
        code = proc.wait()
        with self.lock:
            if self.policy_proc is proc:
                self.policy_proc = None
                self.message = 'policy {} ended (exit code {})'.format(self.policy_mode, code)
                self.policy_mode = ''
                self.policy_status = ''

    def _shutdown_policy(self, _request):
        with self.lock:
            proc = self.policy_proc
        if proc is None:
            return TriggerResponse(False, 'no policy is running')
        os.killpg(proc.pid, signal.SIGINT)
        return TriggerResponse(True, 'shutting down the policy')

    def _on_shutdown(self):
        for proc in (self.policy_proc, self.train_proc):
            if proc is not None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except OSError:
                    pass

    # -- status -------------------------------------------------------------------

    def publish(self):
        with self.lock:
            msg = PolicyStatus()
            msg.training = self.train_proc is not None
            msg.train_job = self.train_job
            msg.train_stage = self.train_stage
            msg.train_error = self.train_error
            msg.results = self.results()
            msg.policy = self.policy_mode
            msg.policy_result = self.policy_result if self.policy_mode else ''
            fresh = self.policy_status_time is not None and rospy.get_time() - self.policy_status_time < 2.0
            msg.policy_status = self.policy_status if fresh else ''
            msg.policy_state = self.policy_state if fresh else ''
            msg.step_mode = self.step_mode if fresh else False
            msg.message = self.message
        self.status_pub.publish(msg)


def main():
    rospy.init_node('kpick_manager')
    manager = KpickManager()
    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        manager.publish()
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break


if __name__ == '__main__':
    main()
