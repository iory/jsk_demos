#!/usr/bin/env python3

from enum import IntEnum
import os

import openai
from openai.openai_object import OpenAIObject
from ros_speak import speak_jp
import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class STATE(IntEnum):
    START = 0
    WAIT_LABEL = 1
    SAVE_PHOTO = 2
    REMOVE_OBJECT = 3
    ASK_CONTINUE = 4
    UPDATE_MODEL = 5


class RegisterObject(object):

    def __init__(self):
        openai.api_key = os.environ['OPENAI_KEY']

        self.speech_msg = None
        self.state = STATE.START

        self.speech_sub = rospy.Subscriber(
            "/speech_to_text", SpeechRecognitionCandidates,
            callback=self.speech_callback, queue_size=1)

    def speech_callback(self, msg):
        self.speech_msg = msg

    def request(self, prompt: str, retry: int = 5) -> OpenAIObject:
        for t in range(retry):
            try:
                rospy.loginfo("OpenAI trying request... {}/{}".format(t+1, retry))
                res: OpenAIObject = openai.Completion.create(
                    model="text-davinci-003",
                    prompt=prompt,
                    temperature=0.0,
                    top_p=1,
                    max_tokens=2048
                )
            except (openai.APIError, openai.error.RateLimitError) as e:
                rospy.logwarn("Caught OpenAI APIError: {}".format(str(e)))
            else:
                break
        return res

    def speak(self, msg):
        rospy.loginfo(msg)
        speak_jp(msg, wait=True)

    def start(self):
        self.speak('物品を登録しますか。')
        self.speech_msg = None

        base = "あなたは日本語の対話システムです。システム(あなた)の「物品を登録しますか？」というメッセージに対してユーザーが返答します。ユーザーの返答を受け取り、ユーザーが物品の登録をすると判断した場合は「1」を、そうでないならば「2」を返答してください。"
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.speech_msg is not None:
                input_text = self.speech_msg.transcript[0]
                self.speech_msg = None
                prompt = base + 'User: "{}" あなたの回答を[1, 2]のどれかのみで返してください。 A: '.format(input_text)
                prompt = " ".join(prompt.split("\n"))
                rospy.loginfo(prompt)
                res = self.request(prompt)
                answer = res.get('choices')[0].get('text').lstrip()
                rospy.loginfo(answer)
                if answer == '1':
                    self.speak('物品を登録しますね')
                    break
                else:
                    self.speak('物品を登録する場合は言ってください。')
            rate.sleep()
        self.state = STATE.WAIT_LABEL

    def reconfirm(self, label_name):
        self.speak('これは「{}」ですか'.format(label_name))
        base = "あなたは日本語の対話システムです。システム(あなた)の「これは{}ですね」というメッセージに対してユーザーが返答します。ユーザーの返答を受け取り、合っている場合には1を合ってない場合には2を、良くわからない返答の場合には3を返してください。".format(label_name)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.speech_msg is not None:
                input_text = self.speech_msg.transcript[0]
                self.speech_msg = None
                prompt = base + 'User: "{}" A: '.format(input_text)
                prompt = " ".join(prompt.split("\n"))
                rospy.loginfo(prompt)
                res = self.request(prompt)
                answer = res.get('choices')[0].get('text').lstrip().rstrip()
                rospy.loginfo(answer)
                if answer.lower() == '1':
                    self.speak('これは「{}」ですね'.format(label_name))
                    self.state = STATE.SAVE_PHOTO
                    break
                elif answer.lower() == '3':
                    self.speak('これは「{}」ですか'.format(label_name))
                else:
                    self.state = STATE.WAIT_LABEL
                    break
            rate.sleep()

    def wait_label(self):
        self.speak('ラベル名を教えてください。')
        self.speech_msg = None

        base = "あなたは日本語の対話システムです。システム(あなた)の「ラベル名を教えてください。」というメッセージに対してユーザーが返答します。ユーザーの返答を受け取り、「ラベル名」に該当する文字列のみを返してください。"
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.speech_msg is not None:
                input_text = self.speech_msg.transcript[0]
                self.speech_msg = None
                prompt = base + 'User: "{}" ラベル名: '.format(input_text)
                prompt = " ".join(prompt.split("\n"))
                rospy.loginfo(prompt)
                res = self.request(prompt)
                answer = res.get('choices')[0].get('text').lstrip().rstrip()
                rospy.loginfo(answer)
                self.reconfirm(answer)
                break
            rate.sleep()

    def save_photo(self):
        self.speak('物体の画像を撮ります。物体を置いてください。')
        self.speech_msg = None

        base = 'あなたは日本語の対話システムです。システム(あなた)の「続いて画像を撮影しますか？」というメッセージに対してユーザーが返答します。ユーザーの返答を受け取り、ユーザーが画像の撮影を続けると判断した場合は「"1"」を、ユーザーが画像の撮影を終了する場合は「"2"」を、ユーザーのメッセージが撮影の続行に関係のない答えならば「"3"」を返答してください。A:のあとに続く回答を["1", "2", "3"]のどれかのみから選んで返答してください。あなたの返答は1文字のみです。 Example 1: User: 続けて A: 1 Example 2: User: 終了 A: 2 Example 3: User: 今日は良い天気です A: 3 Example 4: User: めちゃあつい A: 3 Example 5: User: foo A: 3 Example 6: User: 撮影して A: 1 Example 7: User: 止めて A: 2 ユーザーの回答は以下です。 User: {} あなたの回答を[1, 2, 3]のどれかで返してください。 A:'
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.speech_msg is not None:
                input_text = self.speech_msg.transcript[0]
                self.speech_msg = None
                prompt = base.format(input_text)
                prompt = " ".join(prompt.split("\n"))
                rospy.loginfo(prompt)
                res = self.request(prompt)
                answer = res.get('choices')[0].get('text').lstrip().rstrip()
                rospy.loginfo(answer)

                if answer.lower() == '1':
                    self.speak('写真を撮ります')
                    self.speak('続いてどうしますか。')
                elif answer.lower() == '2':
                    self.speak('写真を撮るのを終了します')
                    self.state = STATE.ASK_CONTINUE
                    break
                else:
                    self.speak('すいません、良くわかりませんでした。物体の画像を取りますか。')
            rate.sleep()

    def ask_continue(self):
        self.speak('他の物体を登録しますか？')
        self.speech_msg = None

        base = 'あなたは日本語の対話システムです。システム(あなた)の「他の物体を登録しますか？」というメッセージに対してユーザーが返答します。ユーザーの返答を受け取り、ユーザーが他の物体を登録すると判断した場合は「"1"」を、ユーザーが終了する場合は「"2"」を、ユーザがモデルを学習する場合には「"2"」を、ユーザーのメッセージが続行に関係のない答えならば「"3"」を返答してください。'
        base += """
Example 1:
User: いいえ
A: 2

Example 2:
User: 終了
A: 2

Example 3:
User: 登録します
A: 1
        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.speech_msg is not None:
                input_text = self.speech_msg.transcript[0]
                self.speech_msg = None
                prompt = base + "User: {} A:".format(input_text)
                prompt = " ".join(prompt.split("\n"))
                rospy.loginfo(prompt)
                res = self.request(prompt)
                answer = res.get('choices')[0].get('text').lstrip().rstrip()
                rospy.loginfo(answer)

                if answer.lower() == '1':
                    self.speak('分かりました。')
                    self.state = STATE.WAIT_LABEL
                    break
                elif answer.lower() == '2':
                    self.speak('終了します')
                    self.state = STATE.UPDATE_MODEL
                    break
                else:
                    self.speak('すいません、良くわかりませんでした。他の物体を登録しますか？')
            rate.sleep()

    def update_model(self):
        self.speak('物体を学習します。時間がかかりますがお待ちください。')
        self.state = STATE.START

    def current_state(self):
        if self.state == STATE.START:
            self.start()
        elif self.state == STATE.WAIT_LABEL:
            self.wait_label()
        elif self.state == STATE.SAVE_PHOTO:
            self.save_photo()
        elif self.state == STATE.ASK_CONTINUE:
            self.ask_continue()
        elif self.state == STATE.UPDATE_MODEL:
            self.update_model()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.current_state()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('register_object')
    parser = RegisterObject()
    parser.run()
