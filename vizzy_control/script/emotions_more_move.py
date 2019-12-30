#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import *
import vizzy_msgs.msg
import woz_dialog_msgs.msg
from std_msgs.msg import Int16, Float64
from time import sleep

class Emotions:
    leftArmPub = rospy.Publisher('/vizzyArmRoutines/left/command', Int16, queue_size=1)
    rightArmPub = rospy.Publisher('/vizzyArmRoutines/right/command', Int16, queue_size=1)
    torsoPub = rospy.Publisher('/vizzy/torso_joint/cmd', Float64, queue_size=1)

    def __init__(self):
	
        rospy.init_node('emotions', anonymous=False)
        rospy.on_shutdown(self.shutdown)

	rate = rospy.Rate(5) # 5hz

        self.gaze_client = actionlib.SimpleActionClient('gaze', vizzy_msgs.msg.GazeAction)
        self.gaze_client.wait_for_server()
        rospy.loginfo("Connected to gaze server")

	self.tts_client = actionlib.SimpleActionClient('gcloud_tts', woz_dialog_msgs.msg.SpeechAction)
        self.tts_client.wait_for_server()
	rospy.loginfo("Connected to GCloud TTS server")

        while not rospy.is_shutdown():
	    self.sequence_more_move()
	    break


    def sequence_more_move(self):
	rospy.loginfo("Started sequence with more movement!")
	sleep(1)
	self.arm_forward()
	sleep(1)
	self.speakCommand(1)
	sleep(2)
	self.home_pose()
	sleep(9)
	self.speakCommand(2)
	self.get_closer()
	sleep(5)
	self.home_pose()
	sleep(5)
	self.fear_emotion()
	self.speakCommand(3)
	sleep(4)
	self.torso_back()
	self.speakCommand(4)
	sleep(4)
	self.home_pose()
	sleep(1)
	self.arm_forward()
	sleep(2)
	self.speakCommand(5)
	sleep(2)
	self.home_pose()
	sleep(7)
	self.get_closer()
	sleep(4)
	self.speakCommand(6)
	sleep(2)
	self.home_pose()
	sleep(16)
	self.speakCommand(7)
	self.sad_emotion_more_move()
	sleep(5) # sad emotion with more movement
	# sleep(9) # sad emotion with more movement
	self.home_pose()
	sleep(2)
	self.speakCommand(8)
	sleep(12)
	self.sad_emotion_less_move()
	sleep(3)
	self.speakCommand(9)
	sleep(5)
	self.home_pose()
	sleep(5)
	self.speakCommand(10)
	sleep(2)
	self.arm_forward()
	sleep(1)
	self.speakCommand(11)
	sleep(2)
	self.home_pose()
	sleep(1)
	self.say_no()
	sleep(4)
	self.angry_emotion()
	self.speakCommand(12)
	sleep(7)
	self.speakCommand(13)
	sleep(7)	
	self.home_pose()
	sleep(10)
	self.surprise_emotion()
	sleep(2)
	self.speakCommand(14)
	sleep(4)
	self.speakCommand(15)
	sleep(5)
	self.surprise_open_emotion()
	self.speakCommand(16)
	sleep(6)
	self.home_pose()
	sleep(6)
	self.arm_forward()
	sleep(1)
	self.speakCommand(17)
	sleep(2)
	self.home_pose()
	sleep(4)
	self.speakCommand(18)
	sleep(3)
	self.happy_emotion_more_move()
	sleep(1)
	self.speakCommand(19)
	sleep(4)
	self.speakCommand(20)
	sleep(6)
	self.speakCommand(21)
	self.happy_emotion_more_move()
	sleep(6)
	self.speakCommand(22)
	self.happy_emotion_more_move()
	sleep(5)
	self.home_pose()


    def sequence_less_move(self):
	rospy.loginfo("Started sequence with less movement!")
	sleep(2)
	self.speakCommand(1)
	sleep(11)
	self.speakCommand(2)
	sleep(10)
	self.fear_emotion()
	self.speakCommand(3)
	sleep(4)
	self.speakCommand(4)
	sleep(4)
	self.home_pose()
	sleep(3)
	self.speakCommand(5)
	sleep(13)
	self.speakCommand(6)
	sleep(18)
	self.speakCommand(7)
	self.sad_emotion_less_move()
	# sleep(5) # sad emotion with more movement
	sleep(9) # sad emotion with more movement
	self.home_pose()
	sleep(2)
	self.speakCommand(8)
	sleep(12)
	self.sad_emotion_less_move()
	sleep(3)
	self.speakCommand(9)
	sleep(5)
	self.home_pose()
	sleep(5)
	self.speakCommand(10)
	sleep(3)
	self.speakCommand(11)
	sleep(11)
	self.angry_emotion()
	self.speakCommand(12)
	sleep(7)
	self.speakCommand(13)
	sleep(7)	
	self.home_pose()
	sleep(10)
	self.surprise_emotion()
	sleep(2)
	self.speakCommand(14)
	sleep(4)
	self.speakCommand(15)
	sleep(5)
	self.speakCommand(16)
	sleep(6)
	self.home_pose()
	sleep(7)
	self.speakCommand(17)
	sleep(7)
	self.speakCommand(18)
	sleep(3)
	self.happy_emotion_less_move()
	sleep(1)
	self.speakCommand(19)
	sleep(4)
	self.speakCommand(20)
	sleep(6)
	self.speakCommand(21)
	sleep(6)
	self.speakCommand(22)
	sleep(5)
	self.home_pose()
	

    def speak(self, message, speed):
	goal = woz_dialog_msgs.msg.SpeechGoal(language="pt_PT", voice="pt-PT-Wavenet-D", message=message, speed=speed)
	self.tts_client.send_goal(goal)
	# client.wait_for_result()    
    
    def speakCommand(self, command):
	phrases = {
	    1: ("<emphasis level='strong'>Porque se está ela a deitar?</emphasis>", 2.5),
	    2: ("Tem algodão na mão?", 2),
	    3: ("<emphasis level='strong'>Não consigo ver!</emphasis>", 2),
	    4: ("<emphasis level='strong'>Ai, oh meu deus!</emphasis>", 4),
	    5: ("<emphasis level='strong'><break time='1s'/>Olha. é o Simba!</emphasis>", 3),
	    6: ("<emphasis level='strong'><prosody rate='medium' pitch='-1st'>É o pai dele</prosody>,<break time='700ms'/>está deitado...</emphasis>", 2),
	    7: ("<prosody rate='slow' pitch='-1st'>Está morto...</prosody>", 2),
	    8: ("<emphasis level='strong'><prosody rate='slow' pitch='-1st'>Oh</prosody><break time='300ms'/><prosody pitch='-1st'>está a chorar...</prosody></emphasis>", 1),
	    9: ("<prosody rate='slow' pitch='-1st'>Coitadinhu...</prosody>", 1.7),
	    10: ("<emphasis level='strong'><prosody pitch='1st'>É um robô como eu!</prosody></emphasis>", 2),
	    11: ("<emphasis level='strong'>Porque é que lhe estão a fazer isto</emphasis>", 3),
	    12: ("<emphasis level='strong'><prosody pitch='1st'> Oh <break time='300ms'/> então! </prosody><break time='500ms'/> Párem! </emphasis>", 2),                                  
	    13: ("<emphasis level='moderate'>Deixem-no em paz</emphasis>", 3),
	    14: ("<emphasis level='strong'><prosody rate='x-high' pitch='1st'>Consegue dar cambalhotas</prosody></emphasis>", 2),
	    15: ("<emphasis level='strong'><prosody pitch='+2st'>Que giiro!</prosody></emphasis>", 1.5),
	    16: ("<emphasis level='strong'><prosody pitch='+1st'>E não cai!</prosody></emphasis>", 3),
	    17: ("<emphasis level='strong'>Quem me dera conseguir fazer o mesmo!</emphasis>", 2),
	    18: ("<emphasis level='strong'><prosody pitch='1st'>Olha</prosody>, é Portugal a jogar!</emphasis>", 3),
	    19: ("<emphasis level='strong'>Que grande golo!</emphasis>", 2),
	    20: ("<emphasis level='strong'>E foi o Éder que marcou!</emphasis>", 3),
	    21: ("<emphasis level='strong'><prosody rate='slow' pitch='2st'>Boa</prosody>, marcámos!</emphasis>", 2),
	    22: ("<emphasis level='strong'><prosody pitch='1st'>Portugal é o melhóóóór!</prosody></emphasis>", 3)
	}
	self.speak(phrases[command][0], phrases[command][1])
 
    
    def sendGazeGoal(self, x, y, z):
	#Look at a predefined 3d point in space
        gaze_point = vizzy_msgs.msg.GazeGoal()
        gaze_point.type = vizzy_msgs.msg.GazeGoal.CARTESIAN
        gaze_point.fixation_point_error_tolerance = 0.01

        #Define the points reference frame.
        ##base_footprint is approximately centered below the front laser,
        ##in the floor. You can check the frames in Rviz.
        ### x axis - forward
        ### y axis - left
        ### z axis - up
        gaze_point.fixation_point.header.frame_id='base_footprint'

        gaze_point.fixation_point.point.x = x
        gaze_point.fixation_point.point.y = y
        gaze_point.fixation_point.point.z = z

        self.gaze_client.send_goal(gaze_point)
	

    def sendGazeHome(self):
	#Home goal makes the robot look forward
        home_goal = vizzy_msgs.msg.GazeGoal()
        home_goal.type = vizzy_msgs.msg.GazeGoal.HOME
        self.gaze_client.send_goal(home_goal)

    def home_pose(self):
	armL = 6
	armR = 6
	torsoRad = 0.0
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeHome()


    def sad_emotion_more_move(self):
	armL = 9
	armR = 9
	torsoRad = 0.3
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeGoal(0.5, 0.0, 0.0)
	sleep(0.8)
	for i in range(3):
	    self.sendGazeGoal(0.5, 0.05, 0.0)
	    sleep(0.8)
	    self.sendGazeGoal(0.5, -0.05, 0.0)
	    sleep(0.8)
	self.sendGazeGoal(0.5, 0.0, 0.0)


    def sad_emotion_less_move(self):
	armL = 9
	armR = 9
	torsoRad = 0.3
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeGoal(0.5, 0.0, 0.0)


    def happy_emotion_more_move(self):
	armL = 8
	armR = 8
	torsoRad = 0.0
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeHome()


    def happy_emotion_less_move(self):
	armL = 7
	armR = 7
	torsoRad = 0.0
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeHome()  
	
    def angry_emotion(self):
	armL = 10
	armR = 10
	torsoRad = 0.3
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeHome()  

    def fear_emotion(self):
	armL = 12
	armR = 11
	torsoRad = -0.1
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeHome()

    def surprise_emotion(self):
	armL = 13
	armR = 13
	torsoRad = 0.0
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeHome()  

    def surprise_open_emotion(self):
	armL = 15
	armR = 15
	torsoRad = 0.0
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeHome()

    def arm_forward(self):
	armL = 14
	armR = 6
	torsoRad = 0.0
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeHome()
				  

    def get_closer(self):
	armL = 6
	armR = 6
	torsoRad = 0.3
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeHome()	
				  

    def torso_back(self):
	torsoRad = -0.2
	self.torsoPub.publish(torsoRad)
	self.sendGazeHome()	

    def say_no(self):
	for i in range(3):
	    self.sendGazeGoal(11.0, 1.2, 0.0)
	    sleep(0.7)
	    self.sendGazeGoal(11.0, -1.2, 0.0)
	    sleep(0.7)
	self.sendGazeHome()

    
    def shutdown(self):
        rospy.loginfo("Shutting emotions script")

        
if __name__ == '__main__':
    try:
        Emotions()
    except rospy.ROSInterruptException:
        rospy.loginfo("Gaze test done")


