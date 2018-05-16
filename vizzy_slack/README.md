slack-ros-pkg
====================

General description
---------------------
ROS packages to enable communication between [ROS](http://wiki.ros.org/) and [Slack](https://slack.com/).

Installation
---------------------
First of all, you have to install [python_slackclient](https://github.com/slackhq/python-slackclient). 

```
pip install python_slackclient
```

Then, install this package :

    cd ~/catkin_ws
    catkin_make --pkg vizzy_slack

Usage
---------------------
Just run :

    rosrun slack_ros slack_ros.py _token:="xoxp-123456789" _channel:="G123456789Q" _username:="ROSbot"
    
You can run in another terminal the test program :

    rosrun slack_ros slack_test.py

Node: slack_ros.py (in package slack_ros)
---------------------
#### Description
This node makes the communication between [ROS](http://wiki.ros.org/) and [Slack](https://slack.com/). You have to enter some parameters to make it works. It listen what you write in [Slack](https://slack.com/) and publish it on a topic. On the other hand, you can write in a topic to make your bot speak in [Slack](https://slack.com/).

#### Parameters
**token** *(string, default: xoxp-123456789)*

You have to place here, the generated token from https://api.slack.com/web#authentication (you can generate a test token for your slack team).


**channel** *(string, default: G1234567Q)*

Enter the channel where you want to speak with your bot. For example, you can type #general if your channel is public. If your channel is private, use https://api.slack.com/methods/groups.list/test to find the correct string.


**username** *(string, default: ros-bot)*

The name given to your bot in [Slack](https://slack.com/).


#### Published Topics
**from_slack_to_ros** *(std_msgs::String)*   

This topic will show you what people say in the selected channel.

#### Subscribed Topics
**from_ros_to_slack** *(std_msgs::String)*   

This topic is used to send a MESSAGE in the selected [Slack](https://slack.com/) channel.

**send_file_to_slack** *(std_msgs::String)*   

This topic is used to send a FILE in the selected [Slack](https://slack.com/) channel.


Node: slack_test.py (in package slack_ros)
---------------------
#### Description
This node is used to test your communication between [ROS](http://wiki.ros.org/) and [Slack](https://slack.com/). The node wait that you write something on [Slack](https://slack.com/), on the correct channel. Then, the bot will answer you in [Slack](https://slack.com/) with : "You just say : XXXXXXX".

#### Parameters

#### Published Topics
**from_ros_to_slack** *(std_msgs::String)*   

This topic will send the answer of the bot in [Slack](https://slack.com/).

#### Subscribed Topics
**from_slack_to_ros** *(std_msgs::String)*   

This topic subscribe what you say in [Slack](https://slack.com/).



### 

In order to use the laucher you will need to set up the env variable SLACK_TOKEN with the token able to acess to the vislab slack
```
export SLACK_TOKEN='xoxp-XXXXXXX'
```


