// This is an automatically generated file.
// Generated from this vizzy_tactile_TactSensorArray.msg definition:
//   Header header
//   TactSensor[] sensorArray
//   
// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_vizzy_tactile_TactSensorArray
#define YARPMSG_TYPE_vizzy_tactile_TactSensorArray

#include <string>
#include <vector>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include "TickTime.h"
#include "std_msgs_Header.h"
#include "geometry_msgs_Vector3.h"
#include "vizzy_tactile_TactSensor.h"

class vizzy_tactile_TactSensorArray : public yarp::os::idl::WirePortable {
public:
  std_msgs_Header header;
  std::vector<vizzy_tactile_TactSensor> sensorArray;

  vizzy_tactile_TactSensorArray() :
    header(),
    sensorArray()
  {
  }

  void clear() {
    // *** header ***
    header.clear();

    // *** sensorArray ***
    sensorArray.clear();
  }

  bool readBare(yarp::os::ConnectionReader& connection) YARP_OVERRIDE {
    // *** header ***
    if (!header.read(connection)) return false;

    // *** sensorArray ***
    int len = connection.expectInt();
    sensorArray.resize(len);
    for (int i=0; i<len; i++) {
      if (!sensorArray[i].read(connection)) return false;
    }
    return !connection.isError();
  }

  bool readBottle(yarp::os::ConnectionReader& connection) YARP_OVERRIDE {
    connection.convertTextMode();
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(2)) return false;

    // *** header ***
    if (!header.read(connection)) return false;

    // *** sensorArray ***
    if (connection.expectInt()!=BOTTLE_TAG_LIST) return false;
    int len = connection.expectInt();
    sensorArray.resize(len);
    for (int i=0; i<len; i++) {
      if (!sensorArray[i].read(connection)) return false;
    }
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::read;
  bool read(yarp::os::ConnectionReader& connection) YARP_OVERRIDE {
    if (connection.isBareMode()) return readBare(connection);
    return readBottle(connection);
  }

  bool writeBare(yarp::os::ConnectionWriter& connection) YARP_OVERRIDE {
    // *** header ***
    if (!header.write(connection)) return false;

    // *** sensorArray ***
    connection.appendInt(sensorArray.size());
    for (size_t i=0; i<sensorArray.size(); i++) {
      if (!sensorArray[i].write(connection)) return false;
    }
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) YARP_OVERRIDE {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(2);

    // *** header ***
    if (!header.write(connection)) return false;

    // *** sensorArray ***
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(sensorArray.size());
    for (size_t i=0; i<sensorArray.size(); i++) {
      if (!sensorArray[i].write(connection)) return false;
    }
    connection.convertTextMode();
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::write;
  bool write(yarp::os::ConnectionWriter& connection) YARP_OVERRIDE {
    if (connection.isBareMode()) return writeBare(connection);
    return writeBottle(connection);
  }

  // This class will serialize ROS style or YARP style depending on protocol.
  // If you need to force a serialization style, use one of these classes:
  typedef yarp::os::idl::BareStyle<vizzy_tactile_TactSensorArray> rosStyle;
  typedef yarp::os::idl::BottleStyle<vizzy_tactile_TactSensorArray> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "Header header\n\
TactSensor[] sensorArray\n\
\n================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n================================================================================\n\
MSG: vizzy_tactile/TactSensor\n\
string frame_id\n\
geometry_msgs/Vector3 force\n\
geometry_msgs/Vector3 displacement\n\
\n================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z";
  }

  // Name the class, ROS will need this
  yarp::os::Type getType() YARP_OVERRIDE {
    yarp::os::Type typ = yarp::os::Type::byName("vizzy_tactile/TactSensorArray","vizzy_tactile/TactSensorArray");
    typ.addProperty("md5sum",yarp::os::Value("40c7988b85a42b61d8d1763f322a0e2b"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif
