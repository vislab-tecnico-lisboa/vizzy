#include <iostream>
#include "ForceReadingThread.h"

ForceReadingThread::ForceReadingThread(yarp::os::Subscriber<TactSensorArray> *my_topic__):semStart(0) {
    setSubscriber(my_topic__);
}
ForceReadingThread::~ForceReadingThread(){}
bool ForceReadingThread::threadInit()
{
    std::cout << "Starting thread1" << std::endl;
    yarp::os::Time::delay(0.01);
    array = new std::vector<TactSensor>();
    return true;
}
void ForceReadingThread::setSubscriber(yarp::os::Subscriber<TactSensorArray> *my_topic__){
    my_topic = my_topic__;
    std::cout << "after set subscriber" << std::endl;
}

void ForceReadingThread::run(){
    while(!isStopping()) {
        //std::cout << "Before reading" << std::endl;
        TactSensorArray reading1Mux;
        my_topic->read(reading1Mux);
        //vizzy_tactile_TactSensorArray d;
        //access d
        //d = reading1Mux->sensorArray;
	//std::cout << "Before reading value" << std::endl;
	int arraySize = reading1Mux.sensorArray.size();
	array->resize(arraySize);
	//std::cout << "Array size: " << arraySize << std::endl;
        *array = reading1Mux.sensorArray;
        geometry_msgs_Vector3 currForce = array->at(0).force;
        //std::cout << "Fx : " << currForce.x << " Fy : " << currForce.y << " Fz : " << currForce.z << std::endl;
    }

}

void ForceReadingThread::get_force(int index, yarp::sig::Vector& force){
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

void ForceReadingThread::threadRelease()
{
    std::cout << "Goodbye from force sensor reading thread" << std::endl;
}
