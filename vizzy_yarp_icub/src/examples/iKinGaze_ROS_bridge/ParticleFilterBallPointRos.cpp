#include <iostream>
#include <yarp/os/all.h>
#include <geometry_msgs_PointStamped.h>

using namespace yarp::os;

int main(int argc, char *argv[]) {
  Network yarp;
  yarp::os::Node    *rosNode;
  rosNode = new yarp::os::Node("/particle_filter_ball_bridge");

  yarp::os::Publisher<geometry_msgs_PointStamped> xd_outputPort;
  bool outputOk_3 = xd_outputPort.topic("/ball_point");
  xd_outputPort.setWriteOnly();

  BufferedPort<Bottle>  receiverBuff1Mux1;
  bool receiver1Mux1Ok = receiverBuff1Mux1.open("/ball_point_yarp");
  receiverBuff1Mux1.setReadOnly();

  
  
 if(!outputOk_3)
  {
    printf("outputOk_3 failed to open\n");
    return -1;
  }


  bool connectSuccess = false;
  while(!connectSuccess)
  {
    printf("\nTrying to connect to /pf3dTracker/data:o ... ");
    connectSuccess = yarp.connect("/pf3dTracker/data:o", receiverBuff1Mux1.getName());
    yarp::os::Time::delay(1);
  }
  
  while(true){
    Bottle *reading1Mux;
    reading1Mux = receiverBuff1Mux1.read(false);
    yarp::os::Time::delay(0.02);
    if (reading1Mux != NULL){
	//std::cout << "point read" << std::endl;
	geometry_msgs_PointStamped & out = xd_outputPort.prepare();
	out.point.x = reading1Mux->get(0).asDouble();
	out.point.y = reading1Mux->get(1).asDouble();
	out.point.z = reading1Mux->get(2).asDouble();
	out.header.frame_id="l_camera_vision_link";
	out.header.stamp.sec=yarp::os::Time::now();
	out.header.stamp.nsec=0.0;
	xd_outputPort.write();
    }
  }
  return 0;
}
