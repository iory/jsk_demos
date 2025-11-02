#!/usr/bin/env python3

import rospy
import subprocess
import shutil
from std_msgs.msg import String, Int32
from threading import Lock


class RecordingController:
    def __init__(self):
        rospy.init_node('recording_controller', anonymous=True)

        # Timeout for topic activity (seconds)
        self.topic_timeout = rospy.get_param('~topic_timeout', 2.0)

        # State management
        self.topics_ready = {
            '/d405/depth/image_rect_raw/compressedDepth': False,
            '/d405/color/image_raw/compressed': False,
            '/t265/odom/sample': False
        }
        self.topic_last_seen = {
            '/d405/depth/image_rect_raw/compressedDepth': None,
            '/d405/color/image_raw/compressed': None,
            '/t265/odom/sample': None
        }
        self.topics_lock = Lock()
        self.is_recording = False
        self.record_process = None

        # Publishers
        self.info_pub = rospy.Publisher('/atom_s3_additional_info', String, queue_size=10)

        # Subscribers for monitored topics
        rospy.Subscriber('/d405/depth/image_rect_raw/compressedDepth', rospy.AnyMsg,
                        self.topic_callback, callback_args='/d405/depth/image_rect_raw/compressedDepth')
        rospy.Subscriber('/d405/color/image_raw/compressed', rospy.AnyMsg,
                        self.topic_callback, callback_args='/d405/color/image_raw/compressed')
        rospy.Subscriber('/t265/odom/sample', rospy.AnyMsg,
                        self.topic_callback, callback_args='/t265/odom/sample')

        # Subscriber for button state
        rospy.Subscriber('/atom_s3_button_state', Int32, self.button_callback)

        # Timer for status publishing and timeout checking
        rospy.Timer(rospy.Duration(0.5), self.publish_status)
        rospy.Timer(rospy.Duration(0.5), self.check_topic_timeouts)

        rospy.loginfo("Recording controller started")

    def topic_callback(self, msg, topic_name):
        """Mark topic as active when message received"""
        with self.topics_lock:
            current_time = rospy.Time.now()
            self.topic_last_seen[topic_name] = current_time
            if not self.topics_ready[topic_name]:
                self.topics_ready[topic_name] = True
                rospy.loginfo(f"Topic {topic_name} is now ready")

    def check_topic_timeouts(self, event):
        """Check if topics have timed out and mark them as not ready"""
        current_time = rospy.Time.now()
        with self.topics_lock:
            for topic_name in self.topics_ready.keys():
                last_seen = self.topic_last_seen[topic_name]
                if last_seen is not None:
                    time_diff = (current_time - last_seen).to_sec()
                    if time_diff > self.topic_timeout:
                        if self.topics_ready[topic_name]:
                            self.topics_ready[topic_name] = False
                            rospy.logwarn(f"Topic {topic_name} timed out (no messages for {time_diff:.1f}s)")

    def all_topics_ready(self):
        """Check if all required topics are publishing"""
        with self.topics_lock:
            return all(self.topics_ready.values())

    def get_disk_space(self):
        """Get available disk space in GB"""
        try:
            stat = shutil.disk_usage('/')
            free_gb = stat.free / (1024 ** 3)
            return f"{free_gb:.1f} GB"
        except Exception as e:
            rospy.logwarn(f"Failed to get disk space: {e}")
            return "Unknown"

    def publish_status(self, event):
        """Publish current status to additional_info topic"""
        disk_space = self.get_disk_space()

        # ANSI color codes
        RED = '\x1b[31m'
        GREEN = '\x1b[32m'
        YELLOW = '\x1b[33m'
        RESET = '\x1b[39m'

        if self.is_recording:
            msg = String()
            msg.data = f"{RED}Recording{RESET}\nFree: {disk_space}"
            self.info_pub.publish(msg)
        elif self.all_topics_ready():
            msg = String()
            msg.data = f"{GREEN}Ready to rec{RESET}\nFree: {disk_space}"
            self.info_pub.publish(msg)
        else:
            msg = String()
            msg.data = f"{YELLOW}Not ready{RESET}\nFree: {disk_space}"
            self.info_pub.publish(msg)

    def button_callback(self, msg):
        """Handle button press events"""
        if msg.data != 1:
            return

        if not self.all_topics_ready() and not self.is_recording:
            rospy.logwarn("Cannot start recording - topics not ready")
            return

        if not self.is_recording:
            # Start recording
            self.start_recording()
        else:
            # Stop recording
            self.stop_recording()

    def start_recording(self):
        """Start the recording launch file"""
        try:
            rospy.loginfo("Starting recording...")
            self.record_process = subprocess.Popen(
                ['roslaunch', 'manip_teaching', 'record.launch'],
                stdout=None,  # Output directly to terminal
                stderr=None   # Output directly to terminal
            )
            self.is_recording = True
            rospy.loginfo("Recording started")
        except Exception as e:
            rospy.logerr(f"Failed to start recording: {e}")
            self.is_recording = False
            self.record_process = None

    def stop_recording(self):
        """Stop the recording launch file"""
        try:
            rospy.loginfo("Stopping recording...")
            if self.record_process is not None:
                self.record_process.terminate()
                self.record_process.wait(timeout=5)
                self.record_process = None
            self.is_recording = False
            rospy.loginfo("Recording stopped")
        except Exception as e:
            rospy.logerr(f"Failed to stop recording: {e}")
            # Force kill if terminate didn't work
            if self.record_process is not None:
                try:
                    self.record_process.kill()
                    self.record_process = None
                except:
                    pass
            self.is_recording = False

    def shutdown(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down recording controller...")
        if self.is_recording:
            self.stop_recording()

    def run(self):
        """Main loop"""
        rospy.on_shutdown(self.shutdown)
        rospy.spin()


if __name__ == '__main__':
    try:
        controller = RecordingController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
