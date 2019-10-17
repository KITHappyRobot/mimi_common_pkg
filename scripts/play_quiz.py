#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
#Title: 工大祭用のクイズを出題するためのROSノード
#Autor: Issei Iida
#Date: 2019/10/15
#Memo:
#---------------------------------------------------------------------

#Python
import pygame
from mutagen.mp3 import MP3 as mp3
import time
import subprocess
#ROS
import rospy


def jtalk(t):
    open_jtalk=['open_jtalk']
    mech=['-x','/var/lib/mecab/dic/open-jtalk/naist-jdic']
    htsvoice=['-m','/usr/share/open_jtalk/voice/mei/mei_happy.htsvoice']
    speed=['-r','0.8']
    pitch=['-fm', '-1.0']
    outwav=['-ow','open_jtalk.wav']
    cmd=open_jtalk+mech+htsvoice+speed+pitch+outwav
    c = subprocess.Popen(cmd,stdin=subprocess.PIPE)
    c.stdin.write(t)
    c.stdin.close()
    c.wait()
    aplay = ['aplay','-q','open_jtalk.wav']
    wr = subprocess.Popen(aplay)

def audioOutput(text):
    play_time = 0
    pygame.mixer.init()
    if text == 'question':
        pygame.mixer.music.load('/home/issei/catkin_ws/src/mimi_common_pkg/mp3_file/Quiz-Question01-1.mp3')
        play_time = mp3('/home/issei/catkin_ws/src/mimi_common_pkg/mp3_file/Quiz-Question01-1.mp3').info.length
    elif text == 'thinking':
        pygame.mixer.music.load('/home/issei/catkin_ws/src/mimi_common_pkg/mp3_file/thinkingtime5.mp3')
        play_time = mp3('/home/issei/catkin_ws/src/mimi_common_pkg/mp3_file/thinkingtime5.mp3').info.length
    elif text == 'answer':
        pygame.mixer.music.load('/home/issei/catkin_ws/src/mimi_common_pkg/mp3_file/Quiz-Results01-1.mp3')
        play_time = mp3('/home/issei/catkin_ws/src/mimi_common_pkg/mp3_file/Quiz-Results01-1.mp3').info.length
        play_time = play_time - 1.5
    elif text == 'end':
        pygame.mixer.music.load('/home/issei/catkin_ws/src/mimi_common_pkg/mp3_file/Phrase05-3.mp3')
        play_time = mp3('/home/issei/catkin_ws/src/mimi_common_pkg/mp3_file/Phrase05-3.mp3').info.length
        play_time = play_time
    pygame.mixer.music.play(1)
    print play_time
    time.sleep(play_time)
    rospy.sleep(0.5)
    pygame.mixer.music.stop()

def main():
    rospy.loginfo('Start play_quiz')
    audioOutput('question')
    text_1 = '私の顔は、何がモチーフになっているでしょうか'
    text_2 = '一番、バナナ'
    text_3 = '二番、みかん'
    text_4 = 'シンキングタイム、スタート！'
    text_5 = 'それでは、正解を発表します！'
    text_6 = '正解は、２番の、みかんです！'
    jtalk(text_1)
    rospy.sleep(5.0)
    jtalk(text_2)
    rospy.sleep(2.0)
    jtalk(text_3)
    rospy.sleep(2.0)
    jtalk(text_4)
    rospy.sleep(3.5)
    audioOutput('thinking')
    rospy.sleep(2.0)
    jtalk(text_5)
    rospy.sleep(3.0)
    audioOutput('answer')
    jtalk(text_6)
    rospy.sleep(3.5)
    audioOutput('end')


if __name__ == '__main__':
    try:
        rospy.init_node('play_quiz', anonymous = True)
        main()
    except rospy.ROSInterruptException:

        pass
