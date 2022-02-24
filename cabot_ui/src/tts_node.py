#!/usr/bin/env python

# Copyright (c) 2020,2022  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# coding=utf-8
#
# tts service
#
# Contributor: Daisuke Sato <daisukes@cmu.edu>
#

import os
import subprocess
import sys
from collections import deque

import rospy
from cabot import util
import cabot_msgs.srv

import requests.exceptions

from ibm_watson import TextToSpeechV1, ApiException
from ibm_watson.websocket import SynthesizeCallback
from ibm_cloud_sdk_core.authenticators import IAMAuthenticator


DEFAULT_RATE = 50
DEFAULT_PITCH = 50
DEFAULT_VOLUME = 50
DEFAULT_LANG = "en"
DEFAULT_VOICE = "male"

queue = deque([])

class SpeechEntry(object):
    def __init__(self,
                 text, 
                 rate = DEFAULT_RATE,
                 pitch = DEFAULT_PITCH,
                 volume = DEFAULT_VOLUME,
                 lang = DEFAULT_LANG,
                 voice = DEFAULT_VOICE):
        self._rate = rate
        self._pitch = pitch
        self._volume = volume
        self._lang = lang
        self._voice = voice
        self._text = text

    def command(self):
        return ["echo 'need to implement command'"]


class IBMCloudEntry(SpeechEntry):
    service = None
    @staticmethod
    def _authenticate():
        if IBMCloudEntry.service is None:
            api_key = rospy.get_param("~iam_apikey")
            service_url = rospy.get_param("~service_url")

            authenticator = IAMAuthenticator(api_key)
            try:
                IBMCloudEntry.service = TextToSpeechV1(authenticator=authenticator)
                IBMCloudEntry.service.set_service_url(service_url)
            except ApiException as error:
                print(error)

    def __init__(self, text, rate, pitch, volume, lang, voice):
        if lang == "ja":
            voice = "ja-JP_EmiVoice" if voice == "female" else "ja-JP_EmiVoice"
        else:
            voice = "en-US_AllisonVoice" if voice == "female" else "en-US_MichaelVoice"
        super(IBMCloudEntry, self).__init__(text,
                                            (50+rate)/100.0,
                                            (pitch-50)*10.0,
                                            (volume-50)/10.0,
                                            lang,
                                            voice)

    @staticmethod
    def _md5hash(text):
        import hashlib
        return hashlib.md5(text.encode("UTF-8")).hexdigest()

    def commands(self):
        filename = "%s.wav" % (IBMCloudEntry._md5hash(self._voice+"/"+self._text))
        if not os.path.isfile(filename):
            rospy.loginfo("audio file does not exist %s", filename)
            IBMCloudEntry._authenticate()
            if IBMCloudEntry.service is None:
                rospy.logerr("TTS Service is not initialized")
                return []

            try:
                service = IBMCloudEntry.service
                response = service.synthesize(self._text, accept='audio/wav',
                                              voice=self._voice).get_result()
                with open(filename, 'wb') as audio_file:
                    audio_file.write(response.content)
            except requests.exceptions.ConnectionError as error:
                print(error)
        else:
            rospy.loginfo("audio file exists %s", filename)

        return [
            [
                "play", filename,
                "tempo", str(self._rate),
                "pitch", str(self._pitch),
                "loudness", str(self._volume)
            ]
        ]


class PauseEntry(SpeechEntry):
    def __init__(self, text, rate, pitch, volume, lang, voice):
        super(PauseEntry, self).__init__(text,
                                         rate,
                                         pitch,
                                         volume,
                                         lang,
                                         voice)

    def commands(self):
        return [["sleep", "1"]]
    
_speaking = False
_speakingProcesses = []

@util.setInterval(0.1)
def checkQueueInterval():
    #rospy.logdebug("waiting (%d)" % (len(queue)))
    checkQueue()

checkQueueInterval()

def checkQueue():
    global _speaking, _speakingProcess
    if _speaking:
        return
    if len(queue) == 0:
        return
    
    command = queue.popleft()
    rospy.logdebug(command)

    _speaking = True
    rospy.loginfo("speaking with command=%s", str(command))
    devnull = open(os.devnull, 'wb') 
    p = subprocess.Popen(command,
                         preexec_fn=os.setsid,
                         stdout=sys.stdout, stderr=sys.stdout,
                         env={"DISPLAY":""})
    _speakingProcesses.append(p)
    rospy.loginfo("waiting speaking finished")
    p.wait()
    _speaking = False
    rospy.loginfo("finished")

def stopSpeak():
    global _speaking, _speakingProcesses
    while len(_speakingProcesses) > 0:
        p = _speakingProcesses.pop()
        result = p.poll()
        if result is None:
            rospy.loginfo("kill %d", p.pid)
            p.kill()
        else:
            rospy.loginfo("poll %d %d", p.pid, result)
    _speaking = False

def getParam(req):
    return {"text": req.text,
            "rate": req.rate,
            "pitch": req.pitch,
            "volume": req.volume,
            "lang": req.lang,
            "voice": req.voice}

def handleSpeak(req):
    if req.force:
        queue.clear()
        stopSpeak()

    param = getParam(req)

    rospy.loginfo(param)
    if param["text"] == "__pose__":
        entry = PauseEntry(**param)
    else:
        entry = IBMCloudEntry(**param)
    
    queue.extend(entry.commands())

    return cabot_msgs.srv.SpeakResponse(True)


if __name__ == "__main__":
    rospy.init_node("speak_node")
    rospy.Service("/speak", cabot_msgs.srv.Speak, handleSpeak)

rospy.spin()
