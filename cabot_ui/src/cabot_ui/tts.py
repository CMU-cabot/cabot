# Copyright (c) 2020  Carnegie Mellon University
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

"""
TTS utility

Author: Daisuke Sato<daisukes@cmu.edu>
"""

import rospy
import cabot_msgs.srv

def speak(text, force=True, pitch=50, volume=50, rate=50, lang="en", priority=50, timeout=2, channels=cabot_msgs.srv.SpeakRequest.CHANNEL_BOTH):
    """speak"""
    try:
        rate = rospy.get_param("/cabot/cabot_menu_node/speech_speed/value")
    except KeyError:
        rospy.loginfo("could not get param '/cabot/cabot_menu_node/speech_speed/value'")

    voice = "female"
    try:
        voice = rospy.get_param("/cabot/cabot_menu_node/speech_voice/value")
        rospy.loginfo("voice %s"% (voice))
        if voice is not "male" or voice is not "female":
            voice = "female"
    except KeyError:
        rospy.loginfo("could not get param '/cabot/cabot_menu_node/speech_voice/value'")

    try:
        #history = open("speech-history.txt", "a+")
        speak_proxy = rospy.ServiceProxy('/speak', cabot_msgs.srv.Speak)
        rospy.loginfo("try to speak %s (v=%s, r=%d, p=%d) %d", text.encode("utf-8"), voice, rate, pitch, force)
        speak_proxy(text, rate, pitch, volume, lang, voice, force, priority, timeout, channels)
        #history.write("%s\n" % (text))
        #history.close()
        rospy.loginfo("speak finished")
    except rospy.ServiceException as error:
        print("Service call failed: %s" % (error))
