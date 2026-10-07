#!/usr/bin/env python3
"""Show the rosbag recording on an AtomS3 and start / stop it with its button.

Forwards ``<manager>/status`` (k_arm_teleop/TeleopStatus) to the AtomS3 over
its USB serial as one line, about 5 Hz, and turns a press of its screen
(``CLICK``) into ``<manager>/start_recording`` or ``stop_recording``, and a
long press (``HOLD``) into ``<manager>/toggle_gripper`` (open <-> close the
real gripper in place of the trigger until the next Start), answering
``R ok`` / ``R err <reason>``. A click while teleop is not
following still starts recording; the AtomS3 then shows NO LABEL, as the
rviz panel warns. The firmware and the line protocol are in
firmware/atoms3_rec_button.

The port (``~port``, default /dev/k_arm_rec_button) is reopened every
second while it is missing, so the AtomS3 can be plugged in at any time.
Without the manager's status the bridge sends ``S none`` and the AtomS3
shows NO TELEOP; without the bridge it shows NO LINK.
"""

import threading

import rospy
import serial
from std_srvs.srv import Trigger

from k_arm_teleop.msg import TeleopStatus


RETRY_PERIOD = 1.0     # [s] between attempts to open a missing port
SEND_PERIOD = 0.2      # [s] between status lines (the AtomS3 shows NO LINK after 2 s)
SERVICE_TIMEOUT = 2.0  # [s] to wait for the manager's recording service
MAX_LINE = 160         # [byte] longest line the firmware reads


def status_line(status):
    """One ``S ...`` line for the AtomS3.

    Parameters
    ----------
    status : k_arm_teleop.msg.TeleopStatus or None
        The manager's latest status, None if it is not heard.

    Returns
    -------
    str
        The line, ending in a newline.
    """
    if status is None:
        return 'S none\n'
    return 'S rec={} dur={:.1f} size={} bags={} phase={} target={} grip={}\n'.format(
        int(status.recording), status.recording_duration, status.current_bag_size,
        status.bag_count, status.phase or '?', status.target or '?',
        status.gripper_override or '-')


def reply_line(ok, message):
    """The ``R ...`` line answering a click.

    Parameters
    ----------
    ok : bool
        Whether the recording was started / stopped.
    message : str
        Why not, when ``ok`` is False.

    Returns
    -------
    str
        The line, ending in a newline, at most ``MAX_LINE`` bytes.
    """
    if ok:
        return 'R ok\n'
    reason = ' '.join(message.split()) or 'failed'
    return ('R err ' + reason)[:MAX_LINE - 1] + '\n'


class RecButtonBridge(object):

    def __init__(self):
        self.port = rospy.get_param('~port', '/dev/k_arm_rec_button')
        self.manager = rospy.get_param('~manager', '/k_arm_teleop_manager').rstrip('/')
        self.status_timeout = rospy.get_param('~status_timeout', 2.0)
        self.lock = threading.Lock()
        self.status = None
        self.status_time = None
        self.device = None
        self.buffer = b''
        self.busy = False
        self.pending_reply = None   # 'R ...' line from the service thread
        rospy.Subscriber(self.manager + '/status', TeleopStatus, self._on_status, queue_size=1)

    def _on_status(self, msg):
        with self.lock:
            self.status = msg
            self.status_time = rospy.get_time()

    def _latest_status(self):
        with self.lock:
            if self.status_time is None or rospy.get_time() - self.status_time > self.status_timeout:
                return None
            return self.status

    # -- serial ----------------------------------------------------------------

    def _open(self):
        try:
            # pyserial's default DTR / RTS keep the ESP32-S3 running; dropping
            # them before opening resets it through the USB-Serial/JTAG
            device = serial.Serial(self.port, 115200, timeout=0, write_timeout=0.5)
        except (serial.SerialException, OSError) as error:
            rospy.logwarn_throttle(30.0, 'rec button: cannot open %s: %s', self.port, error)
            return
        self.device = device
        self.buffer = b''
        rospy.loginfo('rec button: opened %s', self.port)
        self._write('?\n')

    def _drop(self, error):
        rospy.logwarn('rec button: lost %s: %s', self.port, error)
        try:
            self.device.close()
        except (serial.SerialException, OSError):
            pass
        self.device = None

    def _write(self, text):
        if self.device is None:
            return
        try:
            self.device.write(text.encode('ascii', 'replace'))
        except serial.SerialTimeoutException:
            rospy.logwarn_throttle(10.0, 'rec button: %s does not take data', self.port)
        except (serial.SerialException, OSError) as error:
            self._drop(error)

    def _read_lines(self):
        try:
            data = self.device.read(self.device.in_waiting or 1)
        except (serial.SerialException, OSError) as error:
            self._drop(error)
            return []
        self.buffer += data
        lines = self.buffer.split(b'\n')
        self.buffer = lines.pop()
        if len(self.buffer) > 4 * MAX_LINE:
            self.buffer = b''
        return [line.strip().decode('ascii', 'replace') for line in lines]

    def _on_line(self, line):
        if line == 'CLICK':
            self._on_click()
        elif line == 'HOLD':
            self._on_hold()
        elif line.startswith('HELLO '):
            rospy.loginfo('rec button: %s', line[len('HELLO '):])
        # anything else is the ESP32's boot log

    # -- recording ---------------------------------------------------------------

    def _on_click(self):
        status = self._latest_status()
        if status is None:
            self._write(reply_line(False, 'no teleop manager'))
            return
        with self.lock:
            if self.busy:
                self._write(reply_line(False, 'busy'))
                return
            self.busy = True
        service = 'stop_recording' if status.recording else 'start_recording'
        if not status.recording and status.phase != 'following':
            rospy.logwarn('rec button: starting to record while teleop is %s: no action labels '
                          'until it follows', status.phase)
        # the service call blocks; keep the status lines going meanwhile
        threading.Thread(target=self._call, args=(service,)).start()

    def _on_hold(self):
        if self._latest_status() is None:
            self._write(reply_line(False, 'no teleop manager'))
            return
        with self.lock:
            if self.busy:
                self._write(reply_line(False, 'busy'))
                return
            self.busy = True
        threading.Thread(target=self._call, args=('toggle_gripper',)).start()

    def _call(self, service):
        name = '{}/{}'.format(self.manager, service)
        try:
            rospy.wait_for_service(name, timeout=SERVICE_TIMEOUT)
            response = rospy.ServiceProxy(name, Trigger)()
            ok, message = response.success, response.message
        except (rospy.ROSException, rospy.ServiceException) as error:
            ok, message = False, str(error)
        if ok:
            rospy.loginfo('rec button: %s: %s', service, message)
        else:
            rospy.logwarn('rec button: %s failed: %s', service, message)
        with self.lock:
            self.busy = False
            self.pending_reply = reply_line(ok, message)

    def spin(self):
        rate = rospy.Rate(50.0)
        last_retry = last_send = 0.0
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if self.device is None and now - last_retry >= RETRY_PERIOD:
                last_retry = now
                self._open()
            if self.device is not None:
                for line in self._read_lines():
                    self._on_line(line)
            with self.lock:
                reply, self.pending_reply = self.pending_reply, None
            if reply is not None:
                self._write(reply)
            if self.device is not None and now - last_send >= SEND_PERIOD:
                last_send = now
                self._write(status_line(self._latest_status()))
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
        if self.device is not None:
            self.device.close()


def main():
    rospy.init_node('rec_button_bridge')
    RecButtonBridge().spin()


if __name__ == '__main__':
    main()
