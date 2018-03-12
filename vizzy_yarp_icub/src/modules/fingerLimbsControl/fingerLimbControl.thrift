# Copyright: (C) 2018 VisLab, Institute for Systems and Robotics,
#                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
# Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
# CopyPolicy: Released under the terms of the GNU GPL v2.0
#
# fingerLimbControl.thrift

service fingerLimbControl_IDL
{
    /**
    * Perform the grab action, closing the hand of the robot, according to the
    * type of grab requested by the user
    * @param type specifies the type of grab, corresponding to the string
             "one" (default) or "two"
    * @return true/false on success/failure
    **/
    bool grab(1:string type);

    /**
    * Perform the release action, opening the hand of the robot
    * @return true/false on success/failure
    **/
    bool release();
}
