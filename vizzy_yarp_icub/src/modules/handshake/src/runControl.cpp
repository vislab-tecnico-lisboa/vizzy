#include <iostream>
#include "runControl.h"

RunControl::RunControl(yarp::os::Subscriber<TactSensorArray> *my_topic__, IEncoders *encs__,
    IPositionControl *pos__):semStart(0), encs(encs__),  pos(pos__){
    setSubscriber(my_topic__);
    
    //NEED TO CHANGE THIS PARAMETERS. DO I EVEN NEED THEM? PROB NOT
    sensor_force.resize(8);
    force_error = 0.5;                      // accepted force error [N]
    finger_set[0] = 5.0-2.5;//+force_error+0.3; // new force setpoints 
    finger_set[1] = 2.5;//+force_error+0.3;
    finger_set[2] = 2.2;//+force_error+0.3;
    finger_force[0] = 0.0; // current force values
    finger_force[1] = 0.0;
    finger_force[2] = 0.0;
    // Set points per sensor
    sensor_set[0]=0.9525;
    sensor_set[1]=0.35;
    sensor_set[2]=0.124;
    sensor_set[3]=0.5469;
    sensor_set[4]=0.7921;
    sensor_set[5]=0.7921;
    sensor_set[6]=0.5367;
    sensor_set[7]=0.7;
    sensor_set[8]=0.65;
    sensor_set[9]=0.1211;
    sensor_set[10]=0.1358;
    joint_inc[0] = 0.0;
    joint_inc[1] = 0.0;
    joint_inc[2] = 0.0;
    joint_max[0] = 128.0;
    joint_max[1] = 125.0;
    joint_max[2] = 170.0;
    inc_max = 180;                           // max joint increment
    //joint_max = 180;                        // max joint value for fingers (min is 0)
    
    controlActive=false;
    naiveSumControl=true;

}
RunControl::~RunControl(){}
bool RunControl::threadInit()
{
    std::cout << "Starting thread1" << std::endl;
    yarp::os::Time::delay(0.01);
    array = new std::vector<TactSensor>();
    int nj=0;
    pos->getAxes(&nj);
    encoders.resize(nj);
    return true;
}
void RunControl::setSubscriber(yarp::os::Subscriber<TactSensorArray> *my_topic__){
    my_topic = my_topic__;
    std::cout << "after set subscriber" << std::endl;
}

void RunControl::run(){
    while(!isStopping()) {

    TactSensorArray reading1Mux;
    my_topic->read(reading1Mux);
    int arraySize = reading1Mux.sensorArray.size();
    array->resize(arraySize);
    *array = reading1Mux.sensorArray;
    geometry_msgs_Vector3 currForce = array->at(0).force;

    /*Get sensor data*/
    encs->getEncoders(encoders.data());

        if(controlActive)
        {

            for(int sensor_i = 0; sensor_i < 8; sensor_i++) {
		        sensor_force[sensor_i]= std::sqrt(array->at(sensor_i).force.z*array->at(sensor_i).force.z+array->at(sensor_i).force.x*array->at(sensor_i).force.x+array->at(sensor_i).force.y*array->at(sensor_i).force.y);           
            }

            if (sensor_force[0] > 3){
                std::cout << "Handshake initialized." << std::endl;
            }
            //Write control code
            yarp::os::Time::delay(0.01);
        }  
    }
}

void RunControl::EnableControl()
{
    controlActive = true;
    std::cout << "Start the grasping control" << std::endl;
}

void RunControl::DisableControl()
{
    controlActive = false;
    std::cout << "Ending the grasping control" << std::endl;
}


void RunControl::get_force(int index, yarp::sig::Vector& force){
    guard.lock();
    if (array->size()>0){
    //std::vector<vizzy_tactile_TactSensor>& vecRef = *array; // vector is not copied here
    geometry_msgs_Vector3 currForce = array->at(index).force;
    force[0]=currForce.x;//array->at(index).displacement.x;
    force[1]=currForce.y;//array->at(index).displacement.y;
    force[2]=currForce.z;//array->at(index).displacement.z;
    //std::cout << "x: " << force[0] << " y: " << force[1] << " z: " << force[2] << std::endl; 
    }
    guard.unlock();
}

void RunControl::threadRelease()
{
    std::cout << "Goodbye from force sensor reading thread" << std::endl;
}