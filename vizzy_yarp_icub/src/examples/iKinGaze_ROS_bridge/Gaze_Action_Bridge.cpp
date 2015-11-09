#include <iostream>
#include <yarp/os/all.h>
#include <geometry_msgs_PointStamped.h>

using namespace yarp::os;

int main(int argc, char *argv[]) {
  Network yarp;
  yarp::os::Node    *rosNode;
  rosNode = new yarp::os::Node("/ros_tests");
  yarp::os::Subscriber<geometry_msgs_Point> xd_inputPort;
  xd_inputPort.setReadOnly();
  bool outputOk = xd_inputPort.topic("/fixation_point_goal_ros");
  BufferedPort<geometry_msgs_Point> xd_outputPort;
  bool outputOk_3 = xd_outputPort.open("/fixation_point_goal_yarp");

  BufferedPort<geometry_msgs_Point>  receiverBuff1Mux1;
  bool receiver1Mux1Ok = receiverBuff1Mux1.open("/fixation_point_internal");
  yarp::os::Publisher<geometry_msgs_PointStamped> outputPort;
  outputPort.setWriteOnly();
  bool outputOk_1 = outputPort.topic("/fixation_point");
  
 if(!outputOk_3)
  {
    printf("outputOk_3 failed to open\n");
    return -1;
  }

  if(!outputOk_1)
  {
    printf("outputOk_1 failed to open\n");
    return -1;
  }
  bool connectSuccess = false;
  while(!connectSuccess)
  {
    printf("\nTrying to connect to /iKinGazeCtrl/x:o ... ");
    connectSuccess = yarp.connect("/iKinGazeCtrl/x:o", receiverBuff1Mux1.getName());
    yarp::os::Time::delay(1);
  }

  connectSuccess = false;
  
  while(!connectSuccess)
  {
    printf("\nTrying to connect to /iKinGazeCtrl/xd:i ... ");
    connectSuccess = yarp.connect(xd_outputPort.getName(),"/iKinGazeCtrl/xd:i");
    yarp::os::Time::delay(1);
  }
  while(true){
    geometry_msgs_Point *reading1Mux1;
    reading1Mux1 = xd_inputPort.read(false);
    if (reading1Mux1 != NULL){
	geometry_msgs_Point & out = xd_outputPort.prepare();
	out = *reading1Mux1;
	xd_outputPort.write();
    }
    geometry_msgs_Point *reading1Mux;
    reading1Mux = receiverBuff1Mux1.read();

    geometry_msgs_PointStamped & out_ = outputPort.prepare();
    out_.point = *reading1Mux;
    out_.header.frame_id="base_link";
    out_.header.stamp.sec=yarp::os::Time::now();
    out_.header.stamp.nsec=0.0;

    outputPort.write();
  }
  return 0;
}
