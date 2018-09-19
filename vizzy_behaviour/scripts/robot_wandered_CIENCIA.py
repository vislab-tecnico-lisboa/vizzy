#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from time import sleep
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import vizzy_msgs.msg
from random import randint
import woz_dialog_msgs.msg
from copy import deepcopy
import math

class WayPoint:
    def __init__(self):
    	self.gaze_frame = "l_camera_link"
    	self.goal = MoveBaseGoal()
    	self.gaze = vizzy_msgs.msg.GazeGoal()
    	self.gaze.type = vizzy_msgs.msg.GazeGoal.CARTESIAN
    	self.gaze.fixation_point_error_tolerance = 0.01
    	self.gaze.fixation_point.header.frame_id='l_camera_link'
    	self.name = ""
    	self.speechString = "silence5s"



class RandomWalker():
    def callback(self, data):
        self.closestPerson = data.point

    def __init__(self):
        rospy.init_node('random_walker', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.closestPerson = Point()
	self.closestPerson.x = 10.0
	self.closestPerson.y = 0.0
	self.closestPerson.z = 1.5

        #Subscribe to closest points
        rospy.Subscriber("/closest_person", PointStamped, self.callback)


        #Create waypoint list
        waypoints = list()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        quadro_ist = WayPoint()
        goal.target_pose.pose = Pose(Point(1.31260204315, -1.05751419067, 0.0), Quaternion(0.0, 0.0, 0.903550998357, -0.428480563583))
        quadro_ist.goal = deepcopy(goal)
        quadro_ist.gaze.fixation_point.point.x = 1.5
        quadro_ist.gaze.fixation_point.point.y = 0.134899605973
        quadro_ist.gaze.fixation_point.point.z = 0.306768901475
        quadro_ist.name = "Bancada ISR HELI"
        quadro_ist.speechString = "Na bancada do  ISR temos muita informação sobre os projectos que estão a ser desenvolvidos"
        waypoints.append(quadro_ist)




        nem_carne_nem_peixe = WayPoint()
        goal.target_pose.pose = Pose(Point(1.31260204315, -1.05751419067, 0.0), Quaternion(0.0, 0.0, 0.903550998357, -0.428480563583))
        nem_carne_nem_peixe.goal = deepcopy(goal)
        nem_carne_nem_peixe.gaze.fixation_point.point.x = 1.5
        nem_carne_nem_peixe.gaze.fixation_point.point.y = 0.134899605973
        nem_carne_nem_peixe.gaze.fixation_point.point.z = 0.306768901475
        nem_carne_nem_peixe.name = "BANCADA ISR HELI2"
        nem_carne_nem_peixe.speechString = "Não se esqueçam de dar laike na minha página de Feicebúk"
        waypoints.append(nem_carne_nem_peixe)



        escadas_estudo = WayPoint()
        goal.target_pose.pose = Pose(Point(1.31260204315, -1.05751419067, 0.0), Quaternion(0.0, 0.0, 0.903550998357, -0.428480563583))
        escadas_estudo.goal = deepcopy(goal)
        escadas_estudo.gaze.fixation_point.point.x = 1.5
        escadas_estudo.gaze.fixation_point.point.y = -0.18826638842
        escadas_estudo.gaze.fixation_point.point.z = -0.234365782261
        escadas_estudo.name = "BANCADA ISR HELI3"
        escadas_estudo.speechString = "Sigam-me no instagrame"
        waypoints.append(escadas_estudo)


        placar_viva_ritmo = WayPoint()
        goal.target_pose.pose = Pose(Point(12.6063251495, -37.9976234436, 0.0), Quaternion(0.0, 0.0, -0.0116487853205, 0.999932150599))
        placar_viva_ritmo.goal =deepcopy(goal)
        placar_viva_ritmo.gaze.fixation_point.point.x = 1.5
        placar_viva_ritmo.gaze.fixation_point.point.y = 0.0146517878217
        placar_viva_ritmo.gaze.fixation_point.point.z = 0.885482357359
        placar_viva_ritmo.name = "POSTERS1"
        placar_viva_ritmo.speechString = "Acho que há aqui posters em que apareço"
        waypoints.append(placar_viva_ritmo)


        placar_aluger = WayPoint()
        goal.target_pose.pose = Pose(Point(12.6063251495, -37.9976234436, 0.0), Quaternion(0.0, 0.0, -0.0116487853205, 0.999932150599))
        placar_aluger.goal = deepcopy(goal)
        placar_aluger.gaze.fixation_point.point.x = 1.5
        placar_aluger.gaze.fixation_point.point.y = 0.0146517878217
        placar_aluger.gaze.fixation_point.point.z = 0.885482357359
        placar_aluger.name = "POSTERS2"
        placar_aluger.speechString = "Há aqui posters interessantes"
        waypoints.append(placar_aluger)



        placar_wc = WayPoint()
        goal.target_pose.pose = Pose(Point(12.6063251495, -37.9976234436, 0.0), Quaternion(0.0, 0.0, -0.708851171985, 0.705358076424))
        placar_wc.goal =deepcopy(goal)
        placar_wc.gaze.fixation_point.point.x = 1.5
        placar_wc.gaze.fixation_point.point.y = -0.255905735466
        placar_wc.gaze.fixation_point.point.z = -0.132903105059
        placar_wc.name = "POSTERS3"
        placar_wc.speechString = "silence5s"
        waypoints.append(placar_wc)


        maquina_comida = WayPoint()
        goal.target_pose.pose = Pose(Point(18.2798576355, -2.99458551407, 0.0), Quaternion(0.0, 0.0, -0.708851171985, 0.705358076424))
        maquina_comida.goal =deepcopy(goal)
        maquina_comida.gaze.fixation_point.point.x = 1.5
        maquina_comida.gaze.fixation_point.point.y = -0.255905735466
        maquina_comida.gaze.fixation_point.point.z = -0.132903105059
        maquina_comida.name = "COMIDA"
        maquina_comida.speechString = "_________Parece ser bom. Mas nao consigo comer."
        waypoints.append(maquina_comida)



        maquina_comida2 = WayPoint()
        goal.target_pose.pose = Pose(Point(18.2798576355, -2.99458551407, 0.0), Quaternion(0.0, 0.0, -0.708851171985, 0.705358076424))
        maquina_comida2.goal =deepcopy(goal)
        maquina_comida2.gaze.fixation_point.point.x = 1.5
        maquina_comida2.gaze.fixation_point.point.y = -0.0830495813139
        maquina_comida2.gaze.fixation_point.point.z = -1.02727840068
        maquina_comida2.name = "COMIDA2"
        maquina_comida2.speechString = "silence5s"
        waypoints.append(maquina_comida2)



        maquina_comida3 = WayPoint()
        goal.target_pose.pose = Pose(Point(-0.830901145935, -55.5218009949, 0.0), Quaternion(0.0, 0.0, -0.77234469238, 0.635203649353))
        maquina_comida3.goal =deepcopy(goal)
        maquina_comida3.gaze.fixation_point.point.x = 1.5
        maquina_comida3.gaze.fixation_point.point.y = 0.623406451651
        maquina_comida3.gaze.fixation_point.point.z = 0.577336209565
        maquina_comida3.name = "ENTRADASALA2"

        waypoints.append(maquina_comida3)



        maquina_coca_cola = WayPoint()
        goal.target_pose.pose = Pose(Point(-0.830901145935, -55.5218009949, 0.0), Quaternion(0.0, 0.0, -0.77234469238, 0.635203649353))
        maquina_coca_cola.goal =deepcopy(goal)
        maquina_coca_cola.gaze.fixation_point.point.x = 1.5
        maquina_coca_cola.gaze.fixation_point.point.y = 0.623406451651
        maquina_coca_cola.gaze.fixation_point.point.z = 0.577336209565
        maquina_coca_cola.name = "ENTRADASALA2_2"
        maquina_coca_cola.speechString = "Na bancada do  ISR temos muita informação sobre os projectos que estão a ser desenvolvidos"
        waypoints.append(maquina_coca_cola)



        maquina_cafe = WayPoint()
        goal.target_pose.pose = Pose(Point(-0.830901145935, -55.5218009949, 0.0), Quaternion(0.0, 0.0, -0.77234469238, 0.635203649353))
        maquina_cafe.goal =deepcopy(goal)
        maquina_cafe.gaze.fixation_point.point.x = 1.5
        maquina_cafe.gaze.fixation_point.point.y = 0.285209446213
        maquina_cafe.gaze.fixation_point.point.z = 0.862935070425
        maquina_cafe.name = "ENTRADASALA2_3"
        maquina_cafe.speechString = "Não se esqueçam de dar laike na minha página de Feicebúk"
        waypoints.append(maquina_cafe)


        placard_multibanco = WayPoint()
        goal.target_pose.pose = Pose(Point(3.39348983765, -17.2220840454, 0.0), Quaternion(0.0, 0.0, -0.0496053911607, 0.998768894774))
        placard_multibanco.goal =deepcopy(goal)
        placard_multibanco.gaze.fixation_point.point.x = 1.5
        placard_multibanco.gaze.fixation_point.point.y = 0.187508077078
        placard_multibanco.gaze.fixation_point.point.z = 0.55854680238
        placard_multibanco.name = "ENTRADAPRINCIPAL"
        placard_multibanco.speechString = "Sigam-me no instagrame"
        waypoints.append(placard_multibanco)


        escadas_seguranca = WayPoint()
        goal.target_pose.pose = Pose(Point(3.39348983765, -17.2220840454, 0.0), Quaternion(0.0, 0.0, -0.0496053911607, 0.998768894774))
        escadas_seguranca.goal =deepcopy(goal)
        escadas_seguranca.gaze.fixation_point.point.x = 1.5
        escadas_seguranca.gaze.fixation_point.point.y = 0.285209446213
        escadas_seguranca.gaze.fixation_point.point.z = 0.862935070425
        escadas_seguranca.name = "ENTRADAPRINCIPAL2"
        escadas_seguranca.speechString = "Na bancada do  ISR temos muita informação sobre os projectos que estão a ser desenvolvidos"
        waypoints.append(escadas_seguranca)


        relogio = WayPoint()
        goal.target_pose.pose = Pose(Point(2.51701402664, 3.42472648621, 0.0), Quaternion(0.0, 0.0, -0.776358520169, 0.630291558059))
        relogio.goal =deepcopy(goal)
        relogio.gaze.fixation_point.point.x = 1.5
        relogio.gaze.fixation_point.point.y = 0.285209446213
        relogio.gaze.fixation_point.point.z = 0.862935070425
        relogio.name = "HELISUB"
        relogio.speechString = "Chegámos à bancada do ISR"

        waypoints.append(relogio)



        tv_elevadores = WayPoint()
        goal.target_pose.pose = Pose(Point(2.51701402664, 3.42472648621, 0.0), Quaternion(0.0, 0.0, -0.776358520169, 0.630291558059))
        tv_elevadores.goal =deepcopy(goal)
        tv_elevadores.gaze.fixation_point.point.x = 1.5
        tv_elevadores.gaze.fixation_point.point.y = 0.285209446213
        tv_elevadores.gaze.fixation_point.point.z = 0.862935070425
        tv_elevadores.name = "HELISUB2"
        tv_elevadores.speechString = "silence5s"
        waypoints.append(tv_elevadores)


        pilhas = WayPoint()
        goal.target_pose.pose = Pose(Point(6.41749477386, 8.2398443222, 0.0), Quaternion(0.0, 0.0, 0.999997911952, 0.00204354865763))
        pilhas.goal =deepcopy(goal)
        pilhas.gaze.fixation_point.point.x = 1.5
        pilhas.gaze.fixation_point.point.y =0.304327019719
        pilhas.gaze.fixation_point.point.z =0.856358791421
        pilhas.name = "Pilhas"
        pilhas.speechString = "_________Isto dá-me fome"
        #waypoints.append(pilhas)

        self.gaze_active = rospy.get_param("~gaze_active", True)
        self.comlicenca_active = rospy.get_param("~comlicenca_active", True)
        self.beep_comlicenca_active = rospy.get_param("~beep_comlicenca_active", False)

	#Initialize
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")

        self.gaze_client = actionlib.SimpleActionClient('gaze', vizzy_msgs.msg.GazeAction)
        self.gaze_client.wait_for_server()
        rospy.loginfo("Connected to gaze server")

        self.speech_client = actionlib.SimpleActionClient('nuance_speech_tts',woz_dialog_msgs.msg.SpeechAction)
        rospy.loginfo("Connected to speech server")
        rospy.loginfo("Starting to wander around")

        while not rospy.is_shutdown():

            home_goal = vizzy_msgs.msg.GazeGoal()
            home_goal.type = vizzy_msgs.msg.GazeGoal.HOME
            self.gaze_client.send_goal(home_goal)

            i = randint(0, len(waypoints)-1)
            print('index: ' + str(i))
            way = waypoints[i]
            print('Moving to '+way.name)
            test = self.move(way)
	    if not test:
	        continue

            print('gazing')
            self.gaze_client.send_goal(way.gaze)

            #self.gaze_client.wait_for_result()
            sleep(1)
            speech_goal = woz_dialog_msgs.msg.SpeechGoal()
            speech_goal.voice = 'Joaquim'
            speech_goal.language = 'pt_PT'
            speech_goal.message ="silence5s"
            self.speech_client.send_goal(speech_goal)
            #self.speech_client.wait_for_result()
            sleep(5)
            speech_goal.message = way.speechString
            self.speech_client.send_goal(speech_goal)
            sleep(2)
            #self.speech_client.wait_for_result()


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


    def move(self, waypoint):

        self.move_base.send_goal(waypoint.goal)
	count_licenca = -1
        #While the robot does not get to the goal position randomly gaze at people
        while not self.move_base.get_state() == GoalStatus.SUCCEEDED and not self.move_base.get_state() == GoalStatus.LOST and not self.move_base.get_state() == GoalStatus.REJECTED and not self.move_base.get_state() == GoalStatus.ABORTED and not self.move_base.get_state() == GoalStatus.PREEMPTED and not rospy.is_shutdown():
            print('moving')
            sleep(0.3)
            #self.move_base.wait_for_result(rospy.Duration(1.0))
	    print(math.sqrt(math.pow(self.closestPerson.x, 2)+math.pow(self.closestPerson.y, 2)))
            #Gaze at people if gaze active
            if self.gaze_active and math.sqrt(math.pow(self.closestPerson.x, 2)+math.pow(self.closestPerson.y, 2)) < 5.0 and self.gaze_active and math.sqrt(math.pow(self.closestPerson.x, 2)+math.pow(self.closestPerson.y, 2)) > 1.5:
		print('gazing at person')
		gaze_person = vizzy_msgs.msg.GazeGoal()
                gaze_person.type = vizzy_msgs.msg.GazeGoal.CARTESIAN
                gaze_person.fixation_point_error_tolerance = 0.01
                gaze_person.fixation_point.header.frame_id="base_footprint"
                gaze_person.fixation_point.point.x = self.closestPerson.x
                gaze_person.fixation_point.point.y = self.closestPerson.y
                gaze_person.fixation_point.point.z = self.closestPerson.z*1.03
                self.gaze_client.send_goal(gaze_person)
                #self.gaze_client.wait_for_result()
                sleep(1.7)
		gaze_person.type = vizzy_msgs.msg.GazeGoal.HOME
                self.gaze_client.send_goal(gaze_person)
		if self.move_base.get_state() == GoalStatus.ABORTED or self.move_base.get_state() == GoalStatus.PREEMPTED or self.move_base.get_state() == GoalStatus.REJECTED:
		    return False


            if self.comlicenca_active and count_licenca < 0:
                if math.sqrt(math.pow(self.closestPerson.x, 2)+math.pow(self.closestPerson.y, 2)) < 2.2:
                    speech_goal = woz_dialog_msgs.msg.SpeechGoal()
		    self.closestPerson.x = 10.0
		    self.closestPerson.y = 0.0
		    self.closestPerson.z = 1.5
                    speech_goal.voice = 'Joaquim'
                    speech_goal.language = 'pt_PT'
		    speech_goal.message ="silence1s"
		    self.speech_client.send_goal(speech_goal)
		    self.speech_client.wait_for_result()
		    if not self.beep_comlicenca_active:
			speech_goal.message = "Com licença"
		    else:
			speech_goal.message = "silence5s"

                    self.speech_client.send_goal(speech_goal)
                    self.speech_client.wait_for_result()
		    sleep(1.0)
		    count_licenca = 10

	    count_licenca = count_licenca-1
	    self.closestPerson.x = 10.0
            self.closestPerson.y = 0.0
            self.closestPerson.z = 1.5

	return True


if __name__ == '__main__':
    try:
        RandomWalker()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
