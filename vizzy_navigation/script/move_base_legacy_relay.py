#!/usr/bin/env python
# Original version of Jorge Santos
# License: 3-Clause BSD
#
# Adapted by Joao Avelino @ ISR/IST - Vislab

import actionlib
import copy

import rospy
import nav_msgs.srv as nav_srvs
import mbf_msgs.msg as mbf_msgs
import move_base_msgs.msg as mb_msgs
from actionlib_msgs.msg import GoalID
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from move_base.cfg import MoveBaseConfig
import time as t


def safety_stop():
    rospy.loginfo("Stopping robot")

    stopped = False

    trials = 0
    while not stopped:

        try:
            current_odom = rospy.wait_for_message("/odom", Odometry, timeout=0.2)

            current_vel = current_odom.twist.twist

            rospy.logdebug("Sending 0 to the wheels!")
            velocity_publisher.publish(vel_msg)
            t.sleep(0.1) 

            if (current_vel.angular.x < 0.001) and (current_vel.angular.y < 0.001) and (current_vel.angular.z < 0.001)\
            and (current_vel.linear.x < 0.001) and (current_vel.linear.y < 0.001) and (current_vel.linear.z < 0.001):
                stopped = True
        except:
            velocity_publisher.publish(vel_msg)
            trials += 1
            if trials == 20:
                break


"""
move_base legacy relay node:
Relays old move_base actions to the new mbf move_base action, similar but with richer result and feedback.
We also relay the simple goal topic published by RViz, the make_plan service and dynamic reconfiguration
calls (note that some parameters have changed names; see http://wiki.ros.org/move_base_flex for details)
"""

def cancel_goal_cb(msg):
    if not mb_as.is_active():
        rospy.logdebug('Hard preempt of move_base_flex')
        mbf_gp_ac.cancel_all_goals()
        mbf_mb_ac.cancel_all_goals()

        #This solves a bug from move_base_flex...
        safety_stop()

    return

def mbf_feedback_cb(feedback):
    mb_as.publish_feedback(mb_msgs.MoveBaseFeedback(base_position=feedback.current_pose))


def simple_goal_cb(msg):
    mbf_mb_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg))
    rospy.loginfo("Relaying move_base_simple/goal pose to mbf")


def mb_execute_cb(msg):
    rospy.loginfo("Relaying legacy move_base goal to mbf")

    mbf_mb_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg.target_pose), feedback_cb=mbf_feedback_cb)

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        state = mbf_mb_ac.simple_state
        status = mbf_mb_ac.get_state()

        if mb_as.is_new_goal_available():
            rospy.loginfo('New goal available. Preempting old goal...')
            mbf_mb_ac.cancel_all_goals()
            mb_as.set_preempted()
            safety_stop()
            return

        if mb_as.is_preempt_requested():
            rospy.loginfo('Preempted move_base_flex')
            mbf_mb_ac.cancel_all_goals()
            mb_as.set_preempted()
            safety_stop()
            return

        if mbf_mb_ac.simple_state == 2:
            result = mbf_mb_ac.get_result()
            rospy.loginfo("MBF execution completed with result [%d]: %s", result.outcome, result.message)
            if result.outcome == mbf_msgs.MoveBaseResult.SUCCESS:
                mb_as.set_succeeded(mb_msgs.MoveBaseResult(), "Goal reached.")
            else:
                mb_as.set_aborted(mb_msgs.MoveBaseResult(), result.message)
            
            return

        rate.sleep()


def make_plan_cb(request):
    mbf_gp_ac.send_goal(mbf_msgs.GetPathGoal(start_pose=request.start, target_pose=request.goal,
                                             use_start_pose = bool(request.start.header.frame_id),
                                             tolerance=request.tolerance))
    rospy.loginfo("Relaying legacy make_plan service to mbf get_path action server")
    mbf_gp_ac.wait_for_result()

    status = mbf_gp_ac.get_state()
    result = mbf_gp_ac.get_result()

    rospy.loginfo("MBF get_path execution completed with result [%d]: %s", result.outcome, result.message)
    if result.outcome == mbf_msgs.GetPathResult.SUCCESS:
        return nav_srvs.GetPlanResponse(plan=result.path)




def mb_reconf_cb(config, level):
    rospy.loginfo("Relaying legacy move_base reconfigure request to mbf")

    if not hasattr(mb_reconf_cb, "default_config"):
        mb_reconf_cb.default_config = copy.deepcopy(config)

    if config.get('restore_defaults'):
        config = mb_reconf_cb.default_config

    mbf_config = copy.deepcopy(config)

    # Map move_base legacy parameters to new mbf ones
    if 'base_local_planner' in mbf_config:  # mbf doesn't allow changing plugins dynamically
        mbf_config.pop('base_local_planner')
    if 'controller_frequency' in mbf_config:
        mbf_config['controller_frequency'] = mbf_config.pop('controller_frequency')
    if 'controller_patience' in mbf_config:
        mbf_config['controller_patience'] = mbf_config.pop('controller_patience')
    if 'max_controller_retries' in mbf_config:
        mbf_config['controller_max_retries'] = mbf_config.pop('max_controller_retries')
    if 'base_global_planner' in mbf_config:
        mbf_config.pop('base_global_planner')  # mbf doesn't allow changing plugins dynamically
    if 'planner_frequency' in mbf_config:
        mbf_config['planner_frequency'] = mbf_config.pop('planner_frequency')
    if 'planner_patience' in mbf_config:
        mbf_config['planner_patience'] = mbf_config.pop('planner_patience')
    if 'max_planning_retries' in mbf_config:
        mbf_config['planner_max_retries'] = mbf_config.pop('max_planning_retries')
    if 'recovery_behavior_enabled' in mbf_config:
        mbf_config['recovery_enabled'] = mbf_config.pop('recovery_behavior_enabled')
    if 'conservative_reset_dist' in mbf_config:
        mbf_config.pop('conservative_reset_dist')  # no mbf equivalent for this!
    if 'clearing_rotation_allowed' in mbf_config:
        mbf_config.pop('clearing_rotation_allowed')  # no mbf equivalent for this!  TODO: shouldn't? don't think so... if you don't want rotation, do not include that behavior! on recovery_behaviors list!
                                                     # Btw, recovery_behaviors is commented out on MoveBase.cfg, so it doesn't apear here
    mbf_drc.update_configuration(mbf_config)
    return config


if __name__ == '__main__':
    rospy.init_node("move_base")

    velocity_publisher = rospy.Publisher('/vizzy/cmd_vel', Twist, queue_size=10)

    vel_msg = Twist()

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    # TODO what happens with malformed target goal???  FAILURE  or INVALID_POSE
    # txt must be:  "Aborting on goal because it was sent with an invalid quaternion"   

    # move_base_flex get_path and move_base action clients
    mbf_mb_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
    mbf_gp_ac = actionlib.SimpleActionClient("move_base_flex/get_path", mbf_msgs.GetPathAction)
    mbf_mb_ac.wait_for_server(rospy.Duration(20))
    mbf_gp_ac.wait_for_server(rospy.Duration(10))

    # move_base_flex dynamic reconfigure client
    mbf_drc = Client("move_base_flex", timeout=10)

    # move_base simple topic and action server
    mb_sg = rospy.Subscriber('move_base_simple/goal', PoseStamped, simple_goal_cb)
    mb_cg = rospy.Subscriber('move_base/cancel', GoalID, cancel_goal_cb)
    mb_as = actionlib.SimpleActionServer('move_base', mb_msgs.MoveBaseAction, mb_execute_cb, auto_start=False)
    mb_as.start()

    # move_base make_plan service
    mb_mps = rospy.Service('~make_plan', nav_srvs.GetPlan, make_plan_cb)

    # move_base dynamic reconfigure server
    mb_drs = Server(MoveBaseConfig, mb_reconf_cb)

    rospy.spin()
