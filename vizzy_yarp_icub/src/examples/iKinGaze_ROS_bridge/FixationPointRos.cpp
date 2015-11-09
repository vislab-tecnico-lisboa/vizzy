#include <iostream>
#include <yarp/os/all.h>
#include <geometry_msgs_PointStamped.h>

using namespace yarp::os;

int main(int argc, char *argv[]) {
  Network yarp;
  yarp::os::Node    *rosNode;
  rosNode = new yarp::os::Node("/gaze_fixation_point_bridge");

  yarp::os::Publisher<geometry_msgs_PointStamped> xd_outputPort;
  bool outputOk_3 = xd_outputPort.topic("/fixation_point");
  xd_outputPort.setWriteOnly();

  BufferedPort<geometry_msgs_Point>  receiverBuff1Mux1;
  bool receiver1Mux1Ok = receiverBuff1Mux1.open("/fixation_point_yarp");
  receiverBuff1Mux1.setReadOnly();
  
 if(!outputOk_3)
  {
    printf("outputOk_3 failed to open\n");
    return -1;
  }


  bool connectSuccess = false;
  while(!connectSuccess)
  {
    printf("\nTrying to connect to /iKinGazeCtrl/x:o ... ");
    connectSuccess = yarp.connect("/iKinGazeCtrl/x:o", receiverBuff1Mux1.getName());
    yarp::os::Time::delay(1);
  }
  
  while(true){
    geometry_msgs_Point *reading1Mux;
    reading1Mux = receiverBuff1Mux1.read(false);
    yarp::os::Time::delay(0.03);
    if (reading1Mux != NULL){
	//std::cout << "point read" << std::endl;
	geometry_msgs_PointStamped & out = xd_outputPort.prepare();
	out.point = *reading1Mux;

	out.header.frame_id="base_link";
	out.header.stamp.sec=yarp::os::Time::now();
	out.header.stamp.nsec=0.0;
	xd_outputPort.write();
    }
  }
  return 0;
}
