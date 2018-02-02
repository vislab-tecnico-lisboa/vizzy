#include "ForceReadingThread.h"

ForceReadingThread::ForceReadingThread(yarp::os::Subscriber<vizzy_tactile_TactSensorArray> *my_topic__):semStart(0) {
    setSubscriber(my_topic__);
}
ForceReadingThread::~ForceReadingThread(){}
bool ForceReadingThread::threadInit()
{
    std::cout << "Starting thread1" << std::endl;
    yarp::os::Time::delay(0.01);
    return true;
}
void ForceReadingThread::setSubscriber(yarp::os::Subscriber<vizzy_tactile_TactSensorArray> *my_topic__){
    my_topic = my_topic__;
    std::cout << "after set subscriber" << std::endl;
}

void ForceReadingThread::run(){
    while(!isStopping()) {
        //std::cout << "Before reading" << std::endl;
        vizzy_tactile_TactSensorArray reading1Mux;
        my_topic->read(reading1Mux);
        //vizzy_tactile_TactSensorArray d;
        //access d
        //d = reading1Mux->sensorArray;
        std::vector<vizzy_tactile_TactSensor> array = reading1Mux.sensorArray;
        currForce = array[0].force;
        //std::cout << "Fx : " << currForce.x << " Fy : " << currForce.y << " Fz : " << currForce.z << std::endl;
    }

}

void ForceReadingThread::get_force(yarp::sig::Vector& force){
    guard.lock();
    force[0]=currForce.x;
    force[1]=currForce.y;
    force[2]=currForce.z;
    guard.unlock();
}

void ForceReadingThread::threadRelease()
{
    std::cout << "Goodbye from force sensor reading thread" << std::endl;
}

