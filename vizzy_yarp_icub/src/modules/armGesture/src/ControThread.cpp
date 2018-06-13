#include <iostream>
#include "ControlThread.h"

ControlThread::ControlThread(yarp::os::Subscriber<TactSensorArray> *my_topic__, IEncoders *encs__,
    IPositionControl *pos__):semStart(0), encs(encs__),  pos(pos__){
    setSubscriber(my_topic__);
}
ControlThread::~ControlThread(){}
bool ControlThread::threadInit()
{
    std::cout << "Starting thread1" << std::endl;
    yarp::os::Time::delay(0.01);
    array = new std::vector<TactSensor>();
    return true;
}
void ControlThread::setSubscriber(yarp::os::Subscriber<TactSensorArray> *my_topic__){
    my_topic = my_topic__;
    std::cout << "after set subscriber" << std::endl;
}

void ControlThread::run(){
    while(!isStopping()) {
        if(controlActive)
        {
            TactSensorArray reading1Mux;
            my_topic->read(reading1Mux);
            int arraySize = reading1Mux.sensorArray.size();
            array->resize(arraySize);
            *array = reading1Mux.sensorArray;
            geometry_msgs_Vector3 currForce = array->at(0).force;

            /*Get sensor data*/

            encs->getEncoders(encoders.data());

            for(int sensor_i = 0; sensor_i < 11; sensor_i++) {
               
                sensor_force[sensor_i]= std::abs(array->at(sensor_i).force.x)+std::abs(array->at(sensor_i).force.y)+std::abs(array->at(sensor_i).force.z);
     
                if (sensor_force[sensor_i]>10){    //just in case some sensor breaks during experiment
                sensor_force[sensor_i]=10;
                }
                if (sensor_force[sensor_i]<0){    //just in case some sensor breaks during experiment
                sensor_force[sensor_i]=0;
                }
                        
            }

            finger_force[0] = sensor_force[0]+sensor_force[1]+sensor_force[2];
            finger_force[1] = sensor_force[3]+sensor_force[4]+sensor_force[5]+sensor_force[6]; 
            finger_force[2] = sensor_force[7]+sensor_force[8]+sensor_force[9]+sensor_force[10];

            for (int finger_i = 0; finger_i < 3; finger_i++)
                {
                    //condition for doing control or keeping
                    if (std::abs(finger_force[finger_i] - finger_set[finger_i]) > force_error)
                    {
                        double temp_pid = pid_finger.calculate(finger_set[finger_i], finger_force[finger_i]);

                        //relation factor between the force and the motor angle
                        joint_inc[finger_i] = temp_pid; // isto não dá para incluir no kp? deve dar pois

                        //conditions for incrmentation - No increment larger than inc_max
                        if (std::abs(joint_inc[finger_i]) > inc_max){
                            joint_inc[finger_i] = inc_max*(joint_inc[finger_i]/std::abs(joint_inc[finger_i]));
                        }

                        //conditions for joint: 0 < angles < joint_max
                        if (encoders[8+finger_i] + joint_inc[finger_i] > joint_max){
                            joint_inc[finger_i] = joint_max-encoders[8+finger_i]; 
                        }
                        if (encoders[8+finger_i] + joint_inc[finger_i] < 0){
                            joint_inc[finger_i] = 0 - encoders[8+finger_i];
                        }
                        //std::cout << "Value: " << encoders[8+finger_i] << "Increment: " << joint_inc[finger_i] << std::endl;      
                    }
                    else
                    {
                        joint_inc[finger_i]=0;
                        //std::cout << "Finger " << finger_i << " is ok!" << std::endl;
                    }
                } // end for

            //change the enconders 8, 9 and 10 
            pos->positionMove(8,encoders[8]+joint_inc[0]);
            pos->positionMove(9,encoders[9]+joint_inc[1]);
            pos->positionMove(10,encoders[10]+joint_inc[2]);


        }  
    
    }

}

void ControlThread::EnableControl()
{
    controlActive = true;
}

void ControlThread::DisableControl()
{
    controlActive = false;
}


void ControlThread::get_force(int index, yarp::sig::Vector& force){
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

void ControlThread::threadRelease()
{
    std::cout << "Goodbye from force sensor reading thread" << std::endl;
}
