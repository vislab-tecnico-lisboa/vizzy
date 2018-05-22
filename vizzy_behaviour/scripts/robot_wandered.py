#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from time import sleep
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import vizzy_msgs.msg
from random import randint
import woz_dialog_msgs.msg
from copy import deepcopy

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
    def __init__(self):
        rospy.init_node('random_walker', anonymous=False)
        rospy.on_shutdown(self.shutdown)


        #Create waypoint list
        waypoints = list()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        quadro_ist = WayPoint()
        goal.target_pose.pose = Pose(Point(6.82583618164, -0.298132061958, 0.0), Quaternion(0.0, 0.0, 0.0562285211913, 0.998417925222))
        quadro_ist.goal = deepcopy(goal)
        quadro_ist.gaze.fixation_point.point.x = 1.5
        quadro_ist.gaze.fixation_point.point.y =-0.161539276911 
        quadro_ist.gaze.fixation_point.point.z = 0.619330521451 
        quadro_ist.name = "Quadro IST"
        quadro_ist.speechString = "_________Quadro interessante"
        waypoints.append(quadro_ist)



        nem_carne_nem_peixe = WayPoint()
        goal.target_pose.pose = Pose(Point(6.8226184845, -0.382613778114, 0.0), Quaternion(0.0, 0.0, 0.994299805782, -0.10662033681))
        nem_carne_nem_peixe.goal = deepcopy(goal)
        nem_carne_nem_peixe.gaze.fixation_point.point.x = 1.5
        nem_carne_nem_peixe.gaze.fixation_point.point.y = -0.601618178873
        nem_carne_nem_peixe.gaze.fixation_point.point.z = 0.224095496349
        nem_carne_nem_peixe.name = "Nem carne nem peixe"
        nem_carne_nem_peixe.speechString = "silence5s"
        waypoints.append(nem_carne_nem_peixe)

        

        escadas_estudo = WayPoint()
        goal.target_pose.pose = Pose(Point(5.77421236038, -0.905279278755, 0.0), Quaternion(0.0, 0.0, 0.977314637185, -0.211792587085))
        escadas_estudo.goal = deepcopy(goal)
        escadas_estudo.gaze.fixation_point.point.x = 1.5
        escadas_estudo.gaze.fixation_point.point.y = -0.19578182639
        escadas_estudo.gaze.fixation_point.point.z = -0.0652612527037
        escadas_estudo.name = "Escadas da sala de estudo"
        escadas_estudo.speechString = "_________Cuidado com as escadas"
        waypoints.append(escadas_estudo)

        
        placar_viva_ritmo = WayPoint()
        goal.target_pose.pose = Pose(Point(12.8644828796, -2.74097704887, 0.0), Quaternion(0.0, 0.0, -0.598005480569, 0.801492011944))
        placar_viva_ritmo.goal =deepcopy(goal) 
        placar_viva_ritmo.gaze.fixation_point.point.x = 1.5
        placar_viva_ritmo.gaze.fixation_point.point.y = -0.173235377375
        placar_viva_ritmo.gaze.fixation_point.point.z = 0.498420726409
        placar_viva_ritmo.name = "Placar do Viva o Ritmo"
        placar_viva_ritmo.speechString = "silence5s"
        waypoints.append(placar_viva_ritmo)

        
        placar_aluger = WayPoint()
        goal.target_pose.pose = Pose(Point(14.8170385361, -2.95126199722, 0.0), Quaternion(0.0, 0.0, -0.761200110734, 0.648517071031))
        placar_aluger.goal = deepcopy(goal)
        placar_aluger.gaze.fixation_point.point.x = 1.5
        placar_aluger.gaze.fixation_point.point.y = 0.0973222810166 
        placar_aluger.gaze.fixation_point.point.z = 0.960639952322 
        placar_aluger.name = "Placar de aluguer"
        placar_aluger.speechString = "_________Olha, pessoas a alugar coisas."
        waypoints.append(placar_aluger)

        

        placar_wc = WayPoint()
        goal.target_pose.pose = Pose(Point(18.2798576355, -2.99458551407, 0.0), Quaternion(0.0, 0.0, -0.708851171985, 0.705358076424))
        placar_wc.goal =deepcopy(goal) 
        placar_wc.gaze.fixation_point.point.x = 1.5
        placar_wc.gaze.fixation_point.point.y = -0.255905735466
        placar_wc.gaze.fixation_point.point.z = -0.132903105059
        placar_wc.name = "Placar do WC"
        placar_wc.speechString = "_________Não consigo ler o que está neste placard"
        waypoints.append(placar_wc)

        
        maquina_comida = WayPoint()
        goal.target_pose.pose = Pose(Point(19.922203064, 12.430390358, 0.0), Quaternion(0.0, 0.0, 0.620217316172, 0.78443003558))
        maquina_comida.goal =deepcopy(goal) 
        maquina_comida.gaze.fixation_point.point.x = 1.5
        maquina_comida.gaze.fixation_point.point.y = 0.104837718987
        maquina_comida.gaze.fixation_point.point.z = 0.791535363655
        maquina_comida.name = "Maquina da comida"
        maquina_comida.speechString = "_________Parece ser bom. Mas não consigo comer."
        waypoints.append(maquina_comida)

        

        maquina_comida2 = WayPoint()
        goal.target_pose.pose = Pose(Point(19.922203064, 12.430390358, 0.0), Quaternion(0.0, 0.0, 0.620217316172, 0.78443003558))
        maquina_comida2.goal =deepcopy(goal) 
        maquina_comida2.gaze.fixation_point.point.x = 1.5
        maquina_comida2.gaze.fixation_point.point.y = -0.0830495813139
        maquina_comida2.gaze.fixation_point.point.z = -1.02727840068
        maquina_comida2.name = "Maquina da comida2"
        maquina_comida2.speechString = "silence5s"
        waypoints.append(maquina_comida2)

        

        maquina_comida3 = WayPoint()
        goal.target_pose.pose = Pose(Point(19.922203064, 12.430390358, 0.0), Quaternion(0.0, 0.0, 0.620217316172, 0.78443003558))
        maquina_comida3.goal =deepcopy(goal) 
        maquina_comida3.gaze.fixation_point.point.x = 1.5
        maquina_comida3.gaze.fixation_point.point.y = 0.00713634985137
        maquina_comida3.gaze.fixation_point.point.z = 0.35562127909
        maquina_comida3.name = "Maquina da comida 3"

        waypoints.append(maquina_comida3)

        

        maquina_coca_cola = WayPoint()
        goal.target_pose.pose = Pose(Point(18.9359931946, 12.5539255142, 0.0), Quaternion(0.0, 0.0, 0.728744349156, 0.684785859647))
        maquina_coca_cola.goal =deepcopy(goal) 
        maquina_coca_cola.gaze.fixation_point.point.x = 1.5
        maquina_coca_cola.gaze.fixation_point.point.y = -0.278452184481
        maquina_coca_cola.gaze.fixation_point.point.z = 0.34810557026
        maquina_coca_cola.name = "Maquina coca-cola"
        maquina_coca_cola.speechString = "_________Olha, coca-cola"
        waypoints.append(maquina_coca_cola)

        

        maquina_cafe = WayPoint()
        goal.target_pose.pose = Pose(Point(18.9359931946, 12.5539255142, 0.0), Quaternion(0.0, 0.0, 0.728744349156, 0.684785859647))
        maquina_cafe.goal =deepcopy(goal) 
        maquina_cafe.gaze.fixation_point.point.x = 1.5
        maquina_cafe.gaze.fixation_point.point.y = -0.0379565481792
        maquina_cafe.gaze.fixation_point.point.z = 0.0700224520074
        maquina_cafe.name = "Maquina do cafe"
        maquina_cafe.speechString = "_________Sou alérgico ao café. E a líquidos no geral."
        waypoints.append(maquina_cafe)

        
        placard_multibanco = WayPoint()
        goal.target_pose.pose = Pose(Point(13.902557373, 13.0574836731, 0.0), Quaternion(0.0, 0.0, 0.695779308409, 0.718255632759))
        placard_multibanco.goal =deepcopy(goal) 
        placard_multibanco.gaze.fixation_point.point.x = 1.5
        placard_multibanco.gaze.fixation_point.point.y = 0.187508077078
        placard_multibanco.gaze.fixation_point.point.z = 0.55854680238
        placard_multibanco.name = "Placard do multibanco"
        placard_multibanco.speechString = "_________Um"
        waypoints.append(placard_multibanco)

        
        escadas_seguranca = WayPoint()
        goal.target_pose.pose = Pose(Point(6.76146888733, 13.4361066818, 0.0), Quaternion(0.0, 0.0, 0.917695145462, -0.397285313087))
        escadas_seguranca.goal =deepcopy(goal) 
        escadas_seguranca.gaze.fixation_point.point.x = 1.5
        escadas_seguranca.gaze.fixation_point.point.y = -0.0755340082393
        escadas_seguranca.gaze.fixation_point.point.z = 0.37065283186
        escadas_seguranca.name = "Escadas da seguranca"
        escadas_seguranca.speechString = "_________Não consigo subir escadas"
        waypoints.append(escadas_seguranca)
    

        relogio = WayPoint()
        goal.target_pose.pose = Pose(Point(7.147149086, 10.979344368, 0.0), Quaternion(0.0, 0.0, -0.0204962519017, 0.999789929764))
        relogio.goal =deepcopy(goal) 
        relogio.gaze.fixation_point.point.x = 1.5
        relogio.gaze.fixation_point.point.y = 0.0836535250107
        relogio.gaze.fixation_point.point.z = 0.905399122557
        relogio.name = "Relogio"
        relogio.speechString = "_________Tic Tac Tic Tac"

        waypoints.append(relogio)

        

        tv_elevadores = WayPoint()
        goal.target_pose.pose = Pose(Point(13.1157531738, 8.31262111664, 0.0), Quaternion(0.0, 0.0, -0.00998402291442, 0.999950158401))
        tv_elevadores.goal =deepcopy(goal) 
        tv_elevadores.gaze.fixation_point.point.x = 1.5
        tv_elevadores.gaze.fixation_point.point.y = 0.140865129254
        tv_elevadores.gaze.fixation_point.point.z = 0.766451514858
        tv_elevadores.name = "TV dos elevadores"
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
        waypoints.append(pilhas)

        gaze_active = rospy.get_param("~gaze_active", False)

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
            self.move(way)
	  
            print('gazing')
            self.gaze_client.send_goal(way.gaze)

            self.gaze_client.wait_for_result()
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

        #While the robot does not get to the goal position randomly gaze at people
        while not self.move_base.get_state() == GoalStatus.SUCCEEDED and not rospy.is_shutdown():
            print('moving')
            sleep(0.5)
            self.move_base.wait_for_result()

            #Gaze at people if gaze active
            #if gaze_active:

	

if __name__ == '__main__':
    try:
        RandomWalker()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
