#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int16
import actionlib

from actionlib_msgs.msg import *

import woz_dialog_msgs.msg
import vizzy_msgs.msg

from scipy.spatial import distance

import threading

from OpenFace.msg import intent_msg
from OpenFace.msg import intent_msg_all

import random


MAX_VALUE_GESTURE =1200
list_of_persons=[]



class point:
 def __init__(self,x,y,or_z,w):

    self.x =x
    self.y =y
    self.or_z =or_z
    self.w =w

 @staticmethod
 def getgoal(self):
    return [0.2,-7,-0.786,0.68]

 

class person_intent:
 def __init__(self,px,py,pz,l,g,ri,bh,bw,bx,by,id):

    self.pose_tra_x =px
    self.pose_tra_y =py
    self.pose_tra_z =pz
    self.looking =l
    self.gesture =g
    self.result_interact =ri
    self.box_h =bh
    self.box_w =bw
    self.box_x =bx
    self.box_y =by
    self.id_model =id





# define state Stop
class Stop(smach.State):
    def __init__(self):
	#print("Stop")
        smach.State.__init__(self,
                             outcomes=['start_roaming', 'continue_stop'],
                             input_keys=['Stop_id_in'],
                             output_keys=['Stop_id_out'])


    def execute(self, userdata):

	global list_of_persons

        #rospy.loginfo('Executing state Stop')
	rospy.sleep(0.1)
	
	start=0
	for new_person in list_of_persons:	    
	    if (new_person.gesture == 1 or  new_person.gesture==2):
		start=1
		userdata.Stop_id_out=-1	
	        return 'start_roaming'
	if(start==0):
	    return 'continue_stop'
	        



# define state Roaming
class Roaming(smach.State):
    def __init__(self):
	#print("init already")
        smach.State.__init__(self,
                             outcomes=['No_one_detected', 'person_detected'],
                             input_keys=['Roaming_id_in'],
                             output_keys=['Roaming_id_out'])

        self.mutex = threading.Lock()
        #self.intent_sub = rospy.Subscriber('continuos_intent_detector_win',intent_msg_all,self.callback)
        self.id_chosen=-1
	self.state=-1
	#self.move_base=[]



    def execute(self, userdata):

	global list_of_persons

        #rospy.loginfo('Executing state ROAMING')
	rospy.sleep(0.005)
	

	#ACTIVE = 1
	'''
	if(self.state != 1):
		#ROAMING to a random point in the map	
		random_index = random.randint(0,len(list_of_points)-1)	
		self.move_base = navigate(list_of_points[random_index].x,list_of_points[random_index].y,list_of_points[random_index].or_z, list_of_points[random_index].w, 'map')
		rospy.sleep(5)
		
	self.state = self.move_base.get_state()
	'''

        #means we have detected a person
	#if(self.state ==1):
	#chose the closest person
	distance=9999
	for new_person in list_of_persons:	    
	    if new_person.pose_tra_z < distance:
	        distance=new_person.pose_tra_z
	        self.id_chosen = new_person.id_model
	

	if self.id_chosen != -1:
	    
	    #userdata.id_person_detected = self.id_chosen
	    userdata.Roaming_id_out = self.id_chosen
	    index_pub.publish(self.id_chosen)
	    self.id_chosen=-1
	    return 'person_detected'
	#means no one was detecte yet
	else:
	    return 'No_one_detected'
	#self.mutex.release()


	





# define state Follow
class Follow(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['following','lost_person_in_follow','reach_person'],
                             input_keys=['Follow_id_in'],
                             output_keys=['Follow_id_out'])
        self.mutex = threading.Lock()


    def execute(self, userdata):
	
	global list_of_persons
        
	#rospy.loginfo('Executing state FOLLOW')
        #first try to find person
        #self.mutex.acquire()

        lost =1
	#print ("start foloowind to person")
        for person in list_of_persons:
	    #print("this is person",person)

            #means we have person and we can follow the person or go to next stage if we are close enought
            #if(person.id_model==userdata.id_person_detected):
	    if(person.id_model==userdata.Follow_id_in):				
                lost=0
                #if we are 1 meter far from vizzy -> approach more
		dist_to_person = distance.euclidean([person.pose_tra_x, person.pose_tra_y, person.pose_tra_z], [0, 0, 0])
                if(dist_to_person >1.75):
		    #print("have not reach vizzy yet")
                    #PUBLISH TO ROS TOPIC HERE to FOLLOW THE PERSON!!!!!!!!!!!1!!!!!!!!!!!!!!!!!!!!
		    #gazeclient(person.pose_tra_x,person.pose_tra_y,person.pose_tra_z)

		    '''
		    goal.fixation_point.point.x = z
		    goal.fixation_point.point.y = x
		    goal.fixation_point.point.z = y+0.3
		    '''
		    
		    #move_base = navigate(person.pose_tra_z,person.pose_tra_x, 0, 'camera_link')



                    userdata.Follow_id_out = userdata.Follow_id_in
                    return 'following'

                #Means we already reach the person
                else:
		    userdata.Follow_id_out = userdata.Follow_id_in		    
		    #print("we have reach vizzy")
                    return 'reach_person'
        #means we did not find the id passed by argument
        if(lost==1):
            userdata.Follow_id_out = -1
	    index_pub.publish(-1)
            rospy.sleep(10)
	    #print("lost id of person")
            return 'lost_person_in_follow'
        #self.mutex.release()


# define state Speak
class Speak(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['fail_speak','speaked'],
                             input_keys=['Speak_id_in'],
                             output_keys=['Speak_id_out'])


    def execute(self, userdata):

	global list_of_persons
	
        #rospy.loginfo('Executing state SPEAK')

        lost =1
        for person in list_of_persons:

	    if(person.id_model==userdata.Speak_id_in):				
                lost=0
                #gazeclient(person.pose_tra_x,person.pose_tra_y,person.pose_tra_z)
                
        #means we did not find the id passed by argument
        if(lost==1):
            index_pub.publish(-1)
            userdata.Speak_id_out = -1
	    #print("lost id of person")
            rospy.sleep(10)
            return 'fail_speak'
        #self.mutex.release()


	#goal = woz_dialog_msgs.msg.SpeechGoal(language="eng-USA", voice="Tom", message="If you want to play, do this gesture")
	#goal = woz_dialog_msgs.msg.SpeechGoal(language="POR-PRT", voice="Joaquim", message="Siga-me por favor")
	ling="por-PRT"
        voi="Joaquim"
        msg="Se gosta de fazer exercício, faça esta gesto"
	
	result_from_action_speak = speak(ling,voi,msg) 
	#print 'RESULTS FROM ACTION!'
	
        if(result_from_action_speak.success==True):
	    userdata.Speak_id_out=userdata.Speak_id_in
            return 'speaked'
        else:
	    userdata.Speak_id_out=-1
	    rospy.sleep(1)
            rospy.sleep(10)
            return 'fail_speak'


# define state Do_gesture
class Do_gesture(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['fail_doing_gesture','succeed_doing_gesture'],
                             input_keys=['Do_gesture_id_in'],
                             output_keys=['Do_gesture_id_out'])


    def execute(self, userdata):
        #rospy.loginfo('Executing state DO_GESTURE')
	
		
	
		

	arm_publisher.publish(1)
        result_from_action_do_gesture = 1

	rospy.sleep(10)

        if(result_from_action_do_gesture==1):
	    userdata.Do_gesture_id_out=userdata.Do_gesture_id_in
            return 'succeed_doing_gesture'
        else:
	    userdata.Speak_id_out=-1
            rospy.sleep(10)
            return 'fail_doing_gesture'

# define state Detect_gesture
class Detect_gesture(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['detecting_gesture','gesture_detected','fail_to_detect_gesture'],
                             input_keys=['Detect_gesture_id_in'],
                             output_keys=['Detect_gesture_id_out'])

        self.list_of_persons=[]
        self.mutex = threading.Lock()
        #self.intent_sub = rospy.Subscriber('continuos_intent_detector_win',intent_msg_all,self.callback)

        self.counter=0
        self.advice=0

    def execute(self, userdata):
        global MAX_VALUE_GESTURE
	global list_of_persons

        #rospy.loginfo('Executing state DETECT_GESTURE')
        lost =1
        for person in list_of_persons:
            #means we are able to detect the gesture
            if(person.id_model== userdata.Detect_gesture_id_in):
                lost=0
                #if we still havent reached the maximum time
                if(self.counter < MAX_VALUE_GESTURE):
			
                    #gazeclient(person.pose_tra_x,person.pose_tra_y,person.pose_tra_z)
         		    
                    #print("THIS IS THE GESTURE",person.gesture)
                    if( person.gesture==3 or person.gesture==4  ):
                        self.counter=0
			#print("detect gesture")
			self.advice=0		
                        return 'gesture_detected'
                    else:
			#print("stil did not detect the gesture, counter is",self.counter)
                        self.counter+=1
			#print 'detecting gesture'
		 	
			'''
			if(person.looking==0 and self.advice==0):
			    self.advice=1
			    ling="eng-USA"
			    voi="Tom"
			    msg="Please look at me and do not forget to do the gesturee"
		
			    result_from_action_speak = speak(ling,voi,msg) 
			'''

			rospy.sleep(0.01)
                        return 'detecting_gesture'

                #maximum time reached
                else:
                    
		    #print("fail to detect gesture when conter was",self.counter)
		    index_pub.publish(-1)
		    self.counter=0
                    self.advice=0
                    rospy.sleep(10)
                    return 'fail_to_detect_gesture'

        #means we did not find the id passed by argument
        if(lost==1):
            index_pub.publish(-1)
            self.advice=0
	    rospy.sleep(10)
            return 'fail_to_detect_gesture'
        #self.mutex.release()



# define state Detect_gesture
class Go_to_point(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['fail_Reach_the_point','point_reached'],
                             input_keys=['Go_point_id_in'],
                             output_keys=['Go_point_id_out'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state GO_TO_POINT')
	
	#PUBLISH TO NOT FOLLOW WITH GAZE ANYMORE
	



	#goal = woz_dialog_msgs.msg.SpeechGoal(language="eng-USA", voice="Tom", message="If you want to play, do this gesture")
	#goal = woz_dialog_msgs.msg.SpeechGoal(language="POR-PRT", voice="Joaquim", message="Siga-me por favor")
	ling="por-PRT"
        voi="Joaquim"
        msg="Boa, então obtenha mais informações sobre o projecto com os meus colegas"
	
	result_from_action_speak = speak(ling,voi,msg) 

	rospy.sleep(10)

	index_pub.publish(-1)
	
	

        userdata.Go_point_id_out = -1
        
        result_from_action_go_to_point = 1

	#point_ = point.getgoal()
	#move_base = navigate(point_[0],point_[1],point_[2], point_[3], 'map')
	#result_from_action_go_to_point = move_base.wait_for_result()
	result_from_action_go_to_point=1
	rospy.sleep(10)
	if(result_from_action_go_to_point==1):
	    rospy.sleep(1)
            return 'point_reached'
        else:
            return 'fail_Reach_the_point'



# EXEMPLO ACTION GAZE

def gazeclient(x,y,z):
	# Creates the SimpleActionClient, passing the type of the action
	# (vizzy_msgs.msg.GazeAction) to the constructor.
	client = actionlib.SimpleActionClient('gaze', vizzy_msgs.msg.GazeAction)

	# Waits until the action server has started up and started
	# listening for goals.
	client.wait_for_server()

	goal = vizzy_msgs.msg.GazeGoal()
	goal.type = vizzy_msgs.msg.GazeGoal.CARTESIAN
	goal.fixation_point_error_tolerance = 0.01
	goal.fixation_point.point.x = z
	goal.fixation_point.point.y = x
	goal.fixation_point.point.z = y+0.3
	goal.fixation_point.header.frame_id='camera_link'
	goal.fixation_point.header.stamp=rospy.get_rostime()

	# Sends the goal to the action server.
	client.send_goal(goal)

	# Waits for the server to finish performing the action.
	#client.wait_for_result()

	return client.get_result()


def speak(lang,voi,msg):
	
	
	client = actionlib.SimpleActionClient('/woz_dialog/speaker', woz_dialog_msgs.msg.SpeechAction)
	client.wait_for_server()

	goal = woz_dialog_msgs.msg.SpeechGoal(language=lang, voice=voi, message=msg)
	#goal = woz_dialog_msgs.msg.SpeechGoal(language="POR-PRT", voice="Joaquim", message="Se quiser fazer exercicio físico faça este gesto")
	client.send_goal(goal)
	client.wait_for_result()
	return client.get_result()



def callback(data):

	global list_of_persons
	#self.mutex.acquire()
	list_of_persons=[]
	for i in range(data.total_models):
	    #print(data.intent_person[i])
	    new_person= person_intent(data.intent_person[i].pose_tra_x,data.intent_person[i].pose_tra_y,data.intent_person[i].pose_tra_z,data.intent_person[i].looking,data.intent_person[i].gesture,data.intent_person[i].result_interact,int(data.intent_person[i].box_h),int(data.intent_person[i].box_w),int(data.intent_person[i].box_x),int(data.intent_person[i].box_y),data.intent_person[i].id_model)
	    list_of_persons.append(new_person)
	#print list_of_persons
	#self.mutex.release()

def navigate(x,y,z_orient,w, frame):
	move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	move_base.wait_for_server(rospy.Duration(5))
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = frame
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.position.z = 0
	goal.target_pose.pose.orientation.z = z_orient
	goal.target_pose.pose.orientation.w = w
	move_base.send_goal(goal)
	return move_base

# main
def main():
    
    
    print("first")

    rospy.init_node('smach_state_machine')
    
    global arm_publisher
    arm_publisher = rospy.Publisher('vizzyArmRoutines/command',Int16,queue_size=100)


    global index_pub
    index_pub = rospy.Publisher('gaze_index',Int16,queue_size=100)	
    
    global list_of_points 
    list_of_points	=[]
    
    list_of_points.append(point(13,18,0.685,0.727))
    list_of_points.append(point(14.63,-4.6,-0.99,0.018))
    #list_of_points.append(point(14.63,-4.6,-0.99,0.018))




    intent_sub = rospy.Subscriber('continuos_intent_detector_win',intent_msg_all,callback)



    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['No_one_detected', 'person_detected','following','lost_person_in_follow','reach_person','fail_speak','speaked','fail_doing_gesture',
        'succeed_doing_gesture','detecting_gesture','gesture_detected','fail_to_detect_gesture','fail_Reach_the_point','point_reached'])

    #variable about the person being detected
    sm.userdata.id_person_detected = -1



    # Open the container
    with sm:
        # Add states to the container

        smach.StateMachine.add('STOP', Stop(),
                               transitions={'continue_stop':'STOP',
                                            'start_roaming':'ROAMING'},

                               remapping={'Stop_id_in':'id_person_detected',
                                        'Stop_id_out':'id_person_detected'})
	

        smach.StateMachine.add('ROAMING', Roaming(),
                               transitions={'No_one_detected':'ROAMING',
                                            'person_detected':'FOLLOW'},

                               remapping={'Roaming_id_in':'id_person_detected',
                                        'Roaming_id_out':'id_person_detected'})

        smach.StateMachine.add('FOLLOW', Follow(),
                               transitions={'following':'FOLLOW',
                                            'lost_person_in_follow':'ROAMING',
                                            'reach_person':'SPEAK'},
                               remapping={'Follow_id_in':'id_person_detected',
                                        'Follow_id_out':'id_person_detected'})

        smach.StateMachine.add('SPEAK', Speak(),
                               transitions={'fail_speak':'ROAMING',
                                            'speaked':'DO_GESTURE'},
                               remapping={'Speak_id_in':'id_person_detected',
                                        'Speak_id_out':'id_person_detected'})
        smach.StateMachine.add('DO_GESTURE', Do_gesture(),
                               transitions={'fail_doing_gesture':'ROAMING',
                                            'succeed_doing_gesture':'DETECT_GESTURE'},
                               remapping={'Do_gesture_id_in':'id_person_detected',
                                        'Do_gesture_id_out':'id_person_detected'})

        smach.StateMachine.add('DETECT_GESTURE', Detect_gesture(),
                               transitions={'detecting_gesture':'DETECT_GESTURE',
                                            'gesture_detected':'GO_TO_POINT',
                                            'fail_to_detect_gesture':'ROAMING'},
                               remapping={'Detect_gesture_id_in':'id_person_detected',
                                        'Detect_gesture_id_out':'id_person_detected'})



        smach.StateMachine.add('GO_TO_POINT', Go_to_point(),
                               transitions={'fail_Reach_the_point':'ROAMING',
                                            'point_reached':'ROAMING'},
                               remapping={'Go_point_id_in':'id_person_detected',
                                        'Go_point_id_out':'id_person_detected'})


    #to visualization

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()	
    sis.stop()

if __name__ == '__main__':
    main()



















