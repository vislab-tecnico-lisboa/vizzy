#include <iostream>
#include <yarp/os/all.h>
#include <geometry_msgs_PointStamped.h>
#include <math.h>
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

    if (reading1Mux != NULL){
	geometry_msgs_PointStamped & out = xd_outputPort.prepare();
	out.point = *reading1Mux;

	out.header.frame_id="base_link";
	double timestamp = (double) Time::now();
	double dummy;
        double frac=modf(timestamp, &dummy);
	out.header.stamp.sec=(int)timestamp;
	out.header.stamp.nsec=(int)round(frac*pow(10,9));
	xd_outputPort.write();
    }
    yarp::os::Time::delay(0.0333);
  }
  return 0;
}
