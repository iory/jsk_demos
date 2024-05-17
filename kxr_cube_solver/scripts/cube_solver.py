#!/usr/bin/env python3

import sys
import rospy
from kxr_cube_solver.srv import SendCommand, SendCommandRequest
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import vision
import robot
import kociemba
import random
from threading import Thread
from ros_speak import speak_jp
from ros_speak import speak_en


valid_language = ['jp', 'en']


tips = {
    'ja': ("ルービックキューブは今年で５０周年です。",
           "ルービックキューブはどんな状態からでも20手以内で解けることが知られています。",
           "ルービックキューブを解くアルゴリズムは複数あるのですが、私はコシエンバのアルゴリズムを使っています。",
           "人間によるルービックキューブの世界最速記録は3.13秒です。",
           "ロボットによる最速記録はなんと0.38秒です。でも手が6本必要です。",
           "ガンロボットは5本の手で5秒以内でルービックキューブを解くことができます。1万円くらいで買えます。"),
    'en': ("The Rubik's Cube is celebrating its 50th anniversary this year.",
           "It is known that a Rubik's Cube can be solved in 20 moves or less from any scrambled position.",
           "There are multiple algorithms to solve the Rubik's Cube, but I use the Kociemba algorithm.",
           "The world record for the fastest human solution of a Rubik's Cube is 3.13 seconds.",
           "The fastest robot record is an astonishing 0.38 seconds, but it requires 6 hands.",
           "The Gan robot can solve a Rubik's Cube in under 5 seconds with 5 hands, and it costs around 10,000 yen."),
}


def send_command(robot):
    srv_name = "/kxr_command_server/send_command"
    try:
        rospy.wait_for_service(srv_name)
        srv_prox = rospy.ServiceProxy(srv_name, SendCommand)
        req = SendCommandRequest()
        req.command = "(progn "+robot.command+")"
        robot.command = ""
        res = srv_prox(req)
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return None


def cube2string(faces):
    indexes = vision.faces2sequence(faces)
    print("indexes:", indexes)
    colorIndex2character = {}
    for k, v in faces.items():
        colorIndex2character[v[4]] = k
    ret = ""
    for i in indexes:
        ret += colorIndex2character[i]
    return ret


class CubeSolverFSM(object):

    def __init__(self):
        self.transitTo("init", 0)
        self.autoTransition = False
        self.start = True
        self.finish = False
        self.scanFaceOrder = ('D','U','L','R','B','F')
        self.solveOperations = None
        self.robot = robot.cubeSolver()
        self.vision = vision.cubeDetector()
        self.faceDetectionCount = 0
        self.maxFaceDetectionCount = 10

        self.lang = rospy.get_param('~language', 'jp')
        if not self.lang in valid_language:
            msg = 'Invalid language {}.'.format(self.lang)
            msg += ' You can use {}.'.format(valid_language)
            rospy.logerr(msg)
            sys.exit(1)

        self.tipsIndex = 0
        self.speakProc = None
        self.moveProc = None

    def isSpeaking(self):
        return self.speakProc.is_alive()

    def speak(self, msg_jp, msg_en, wait=True):
        rospy.loginfo('Speak "{}"'.format(msg_jp))
        if self.lang == 'jp':
            self.speakProc = Thread(
                target=speak_jp,
                args=(msg_jp, 'robotsound_jp', 1, wait))
        elif self.lang == 'en':
            self.speakProc = Thread(
                target=speak_en,
                args=(msg_en, 'robotsound', 1, wait))
        self.speakProc.start()

    def move(self):
        self.moveProc = Thread(target=send_command, args=(self.robot,))
        self.moveProc.start()

    def run(self, frame):
        # face detection
        face = self.vision.detectFace(frame)

        msgs = ("state="+self.state+":"+str(self.index)+"/"+str(self.maxIndex),
                "autoTransition="+str(self.autoTransition),
                "finish="+str(self.finish))
        self.vision.drawInfo(frame, face, msgs)
        cv2.imshow('Video', frame)

        if self.state == "init":
            if self.start:
                self.robot.initDemo()
                self.move()
                self.speak("ルービックキューブを手の上に置いてください。",
                           "Please place the Rubik's Cube on your hand.")
                self.start = False
            elif not self.isMoving():
                self.finish = True
        if self.state == "start":
            if self.start:
                self.speak("まずは、ルービックキューブの状態を見てみます。",
                           "First, let's take a look at the state of the Rubik's Cube.")
                self.robot.startDemo()
                self.move()
                self.vision.initFaces()
                self.robot.initGraspingFaces()
                self.tipsIndex = 0
                self.start = False
            elif not self.isMoving():
                self.finish = True
        elif self.state == "scan":
            if self.index == self.maxIndex:
                if self.start:
                    self.robot.finishScan()
                    self.move()
                    self.start = False
                elif not self.isMoving():
                    self.finish = True
            else:
                faceId = self.scanFaceOrder[int(self.index / 2)]
                if self.index % 2 == 0:
                    if self.start:
                        self.robot.lookAt(faceId)
                        self.move()
                        self.faceDetectionCount = 0
                        self.start = False
                    elif not self.isMoving():
                        self.finish = True
                else:
                    if vision.checkFace(face):
                        if self.vision.faces[faceId] == face:
                            self.faceDetectionCount += 1
                        else:
                            self.faceDetectionCount = 0
                        if self.faceDetectionCount >= self.maxFaceDetectionCount:
                            self.finish = True
                        self.vision.setFace(faceId, face)
                    else:
                        self.faceDetectionCount = 0
                    # print("faceDetectionCount=",self.faceDetectionCount)
        elif self.state == "solve":
            if self.start:
                if not self.isSpeaking():
                    r = random.random()
                    if r > 0.5:
                        if self.tipsIndex < len(tips):
                            self.speak(tips['ja'][self.tipsIndex],
                                       tips['en'][self.tipsIndex])
                            self.tipsIndex += 1
                        else:
                            self.speak("あと"+str(len(self.solveOperations)-self.index)+"手です。",
                                       "There are "+str(len(self.solveOperations)-self.index)+" moves left.")
                # self.robot.adjustPosition()
                # self.move()
                self.robot.solveOneStep(self.solveOperations[self.index])
                self.move()
                self.start = False
            if not self.isMoving():
                self.finish = True
        elif self.state == "end":
            if not self.finish:
                self.speak("完成しました", "It's completed.")
                self.robot.finishDemo()
                send_command(self.robot)
                self.autoTransition = False
                self.finish = True

        if self.autoTransition:
            self.stateTransition()

    def isMoving(self):
        return self.moveProc.is_alive()

    def transitTo(self, state, maxIndex=0):
        self.state = state
        self.index = 0
        self.maxIndex = maxIndex
        self.start = True

    def stateTransition(self):
        if not self.finish:
            return
        self.finish = False
        if self.state == "init":
            self.transitTo("start")
        elif self.state == "start":
            self.transitTo("scan", len(self.scanFaceOrder)*2)
        elif self.state == "scan":
            if self.index == self.maxIndex:
                if self.vision.checkCube(self.vision.faces):
                    print("faces:", self.vision.faces)
                    s = cube2string(self.vision.faces)
                    print("cube state:", s)
                    valid = True
                    try:
                        s = kociemba.solve(s)
                    except ValueError as e:
                        print(e)
                        print("detected cube state is invalid, rescan is necessary")
                        self.speak("見間違えたみたいです。もう一回みてみますね。",
                                   "It seems I made a mistake. Let me check again.")
                        self.index = 0
                        self.start = True
                        valid = False
                    if valid:
                        self.solveOperations = s.split(' ')
                        print("solution:",self.solveOperations)
                        self.speak(
                            "解き方がわかりました｡"+str(len(self.solveOperations))+"手で解けます。",
                            "I have figured out the solution. It can be solved in "+str(len(self.solveOperations))+" moves.")
                        self.transitTo("solve", len(self.solveOperations)-1)
                else:
                    print("detected cube state is invalid, rescan is necessary")
                    self.speak(
                        "見間違えたみたいです。もう一回みてみますね。",
                        "It seems I made a mistake. Let me check again.")
                    self.index = 0
                    self.start = True
            else:
                self.index += 1
                self.start = True
        elif self.state == "solve":
            if self.index == self.maxIndex:
                self.transitTo("end",0)
            else:
                self.index += 1
                self.start = True
        elif self.state == "end":
            self.transitTo("start")

fsm = CubeSolverFSM()


def image_cb(msg):
    bridge = CvBridge()
    frame = None
    tmp = vision.cubeDetector()
    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        fsm.run(frame)

        key = cv2.waitKey(10) & 0xff
        if key == ord('e'):
            fsm.robot.addCommand("(send *ri* :servo-on)")
            fsm.robot.addCommand("(init-pose)")
            send_command(fsm.robot)
            fsm.robot.gripperR.angle = 0
            fsm.robot.gripperL.angle = 0
            fsm.autoTransition = False
            fsm.finish = True
            fsm.state = "end"
        elif key == ord('n'):
            fsm.stateTransition()
        elif key == ord('x'):
            fsm.robot.addCommand("(send *ri* :angle-vector (send *robot* :init-pose))")
            fsm.robot.addCommand("(send *ri* :wait-interpolation)")
            fsm.robot.addCommand("(send *ri* :servo-off)")
            send_command(fsm.robot)
        elif key == ord('a'):
            fsm.autoTransition = not fsm.autoTransition
        elif key == ord('s'):
            tmp.saveROIs(frame)
    except CvBridgeError as e:
        print(e)
        return


if __name__ == "__main__":
    rospy.init_node("cube_solver")
    rospy.Subscriber("~image_in", Image, image_cb, queue_size=1)
    rospy.spin()
