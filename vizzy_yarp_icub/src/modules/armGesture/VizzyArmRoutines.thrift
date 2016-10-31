# Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
# Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
#                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
# Author: Pedro Vicente <pvicente@isr.ist.utl.pt>
# CopyPolicy: Released under the terms of the GNU GPL v2.0
#
# arucoboard.thrift

/** struct Bottle { }
* (
* yarp.name = "yarp::os::Bottle"
* yarp.includefile="yarp/os/Bottle.h"
* )
*/

service VIZZYARMROUTINES_IDL
{

  /**
  * Update the state text file.
  * 
  * @return true/false on success/failure
  */
  bool update();
  
  /**
  * Quit the module.
  * @return true/false on success/failure
  */
  bool quit();  
}
