#!/usr/bin/env python

# Copyright 2020 Carnegie Mellon University
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

import rospy
import std_msgs.msg
import subprocess
import cabot_msgs.srv
import json
import signal
import os
from os.path import join, dirname

from collections import deque
import threading

from ibm_watson import TextToSpeechV1, ApiException
from ibm_watson.websocket import SynthesizeCallback
import requests.exceptions


# https://stackoverflow.com/questions/5179467/equivalent-of-setinterval-in-python
# from here
def setInterval(interval, times = -1):
    # This will be the actual decorator,
    # with fixed interval and times parameter
    def outer_wrap(function):
        # This will be the function to be
        # called
        def wrap(*args, **kwargs):
            stop = threading.Event()

            # This is another function to be executed
            # in a different thread to simulate setInterval
            def inner_wrap():
                i = 0
                while i != times and not stop.isSet():
                    stop.wait(interval)
                    function(*args, **kwargs)
                    i += 1

            t = threading.Timer(0, inner_wrap)
            t.daemon = True
            t.start()
            return stop
        return wrap
    return outer_wrap
# to here

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
    

#
# spd-say server could easily die
# so recommend to use espeak
#
class SPDSayEntry(SpeechEntry):
    def __init__(self, text, rate, pitch, volume, lang, voice):
        voice = "female1" if voice == "female" else "male1"
        super(SPDSayEntry, self).__init__(text,
                                          (rate - 50) * 2,
                                          (pitch - 50) * 2,
                                          (volume - 50) * 2,
                                          lang,
                                          voice)
        
    def commands(self):
        return [["spd-say", self._text,
                "-r", str(self._rate),
                "-p", str(self._pitch),
                "-i", str(self._volume),
                "-l", self._lang,
                "-t", self._voice,
                "-w" # wait
                ]]

#
# robot voice
#
class EspeakEntry(SpeechEntry):
    def __init__(self, text, rate, pitch, volume, lang, voice):
        super(EspeakEntry, self).__init__(text,
                                          80 + rate * 1.9,
                                          pitch * 0.99,
                                          volume * 2,
                                          lang,
                                          voice)

    def commands(self):
        return [["espeak", self._text,
                "-s", str(self._rate),
                "-p", str(self._pitch),
                "-a", str(self._volume),
                #"-l", self._lang,
                #"-t", self._voice,
                ]]


class PicoEntry(SpeechEntry):
    def __init__(self, text, rate, pitch, volume, lang, voice):
        super(PicoEntry, self).__init__(text,
                                          (50+rate)/100.0,
                                          (pitch-50)*10.0,
                                          (volume-50)/10.0,
                                          lang,
                                          voice)
    def escapedText(self):
        return self._text.replace("'", "'\"'\"'")
        
    count=0
    def commands(self):
        PicoEntry.count = (PicoEntry.count + 1) % 10
        filename = "temp%d.wav" % (PicoEntry.count)
        #text = self.escapedText()
        text = self._text
        return [
            [
                "pico2wave", 
                "--wave", filename,
                text
            ],
            [
                "play", filename,
                "tempo", str(self._rate),
                "pitch", str(self._pitch),
                "loudness", str(self._volume)
            ]
        ]

class IBMCloudEntry(SpeechEntry):
    service = None
    @staticmethod
    def _authenticate():
        if IBMCloudEntry.service is None:
            api_key = rospy.get_param("~iam_apikey")
            try:
                IBMCloudEntry.service = TextToSpeechV1(
                    url='https://stream.watsonplatform.net/text-to-speech/api',
                    iam_apikey=api_key)
            except ApiException as error:
                print error

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
        return hashlib.md5(text).hexdigest()

    def commands(self):
        filename = "%s.wav" % (IBMCloudEntry._md5hash(self._voice+"/"+self._text))
        print filename
        if not os.path.isfile(filename):
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

@setInterval(0.1)
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
    rospy.logdebug("speaking")
    devnull = open(os.devnull, 'wb') 
    p = subprocess.Popen(command,
                         preexec_fn=os.setsid,
                         stdout=devnull, stderr=devnull)
    _speakingProcesses.append(p)
    p.wait()
    _speaking = False
    rospy.logdebug("finished")

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
    #queue.append(SPDSayEntry(**param))
    #queue.append(EspeakEntry(**param))

    rospy.loginfo(param)
    if param["text"] == "__pose__":
        entry = PauseEntry(**param)
    else:
        #entry = PicoEntry(**param)
        entry = IBMCloudEntry(**param)
    
    queue.extend(entry.commands())

    return cabot_msgs.srv.SpeakResponse(True)


if __name__ == "__main__":
    rospy.init_node("speak_node")
    lang = rospy.get_param("~language", "en")
    rospy.Service("/speak", cabot_msgs.srv.Speak, handleSpeak)

rospy.spin()
