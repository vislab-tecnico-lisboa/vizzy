/*
 * Copyright: (C) 2015 VisLab, Institute for Systems and Robotics,
 *            Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "Ros2YarpModule.h"

bool Ros2YarpModule::configure(ResourceFinder &rf)
{
    moduleName = rf.check("name",Value("ros2yarp")).asString();
    yarp::os::RFModule::setName(moduleName.c_str());

    mydelay = rf.check("delay",Value(0.05)).asDouble();
    inputImagePortName = rf.check("ros_topic_name",Value("/vizzy/l_camera/image_rect_color")).asString();

    yInfo("refresh delay set to %f", mydelay);

    rosNode = new yarp::os::Node("/ros2yarp_node"); // can be any string

    inputImagePort.setReadOnly();
    bool validInput = inputImagePort.topic(inputImagePortName.c_str());
    if (!validInput)
    {
        yError("could not set ROS topic in input image port");
        return false;
    }

    outputImagePortName = "/" + inputImagePortName + "/rgb:o";
    bool validOutput = outputImagePort.open(outputImagePortName.c_str());
    if (!validOutput)
    {
        yError("could not open output YARP port");
        return false;
    }

    // start clock
    t = yarp::os::Time::now();

    return true;
}

bool Ros2YarpModule::interruptModule()
{
    outputImagePort.interrupt();
    return true;
}

bool Ros2YarpModule::close()
{
    outputImagePort.close();
    return true;
}

double Ros2YarpModule::getPeriod()
{
    return 0.0; // sync with incoming data
}

bool Ros2YarpModule::updateModule()
{
    // if rgb:o is not connected to anything yet, print a message periodically
    if (outputImagePort.getOutputCount() < 1)
    {
        double t0 = yarp::os::Time::now();
        if ((t0 - t) > 10.0)
        {
            yInfo("please connect %s to the input port of an external module...",
                  outputImagePortName.c_str());
            t = t0;
        }
    }
    else // now rgb:o has an output connection -> propagate it
    {
        sensor_msgs_Image *mux;
        mux = inputImagePort.read(false);
        yarp::os::Time::delay(mydelay);
        if (mux != NULL)
        {
            ImageOf<PixelRgb> &outYarp = outputImagePort.prepare();
            // http://stackoverflow.com/a/4254644/1638888
            char *dataYarp = reinterpret_cast<char*>(&(mux->data)[0]);
            outYarp.setExternal(dataYarp,
                                mux->width,
                                mux->height);
            outputImagePort.write();

            // also print a message periodically
            double t0 = yarp::os::Time::now();
            if ((t0 - t) > 10.0)
            {
                yInfo("bridge from ROS topic %s to YARP port %s running successfully",
                     inputImagePortName.c_str(),
                     outputImagePortName.c_str());
                t = t0;
            }
        }
    }

    return true;
}
