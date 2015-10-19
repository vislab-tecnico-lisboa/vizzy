// This is an automatically generated file.
// Generated from this geometry_msgs_PointStamped.msg definition:
//   # This represents a Point with reference coordinate frame and timestamp
//   Header header
//   Point point
//   
// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_geometry_msgs_PointStamped
#define YARPMSG_TYPE_geometry_msgs_PointStamped

#include <string>
#include <vector>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include "TickTime.h"
#include "Header.h"
#include "geometry_msgs_Point.h"

class geometry_msgs_PointStamped : public yarp::os::idl::WirePortable {
public:
  Header header;
  geometry_msgs_Point point;

  geometry_msgs_PointStamped() {
  }

  bool readBare(yarp::os::ConnectionReader& connection) {
    // *** header ***
    if (!header.read(connection)) return false;

    // *** point ***
    if (!point.read(connection)) return false;
    return !connection.isError();
  }

  bool readBottle(yarp::os::ConnectionReader& connection) {
    connection.convertTextMode();
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(2)) return false;

    // *** header ***
    if (!header.read(connection)) return false;

    // *** point ***
    if (!point.read(connection)) return false;
    return !connection.isError();
  }

  bool read(yarp::os::ConnectionReader& connection) {
    if (connection.isBareMode()) return readBare(connection);
    return readBottle(connection);
  }

  bool writeBare(yarp::os::ConnectionWriter& connection) {
    // *** header ***
    if (!header.write(connection)) return false;

    // *** point ***
    if (!point.write(connection)) return false;
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(2);

    // *** header ***
    if (!header.write(connection)) return false;

    // *** point ***
    if (!point.write(connection)) return false;
    connection.convertTextMode();
    return !connection.isError();
  }

  bool write(yarp::os::ConnectionWriter& connection) {
    if (connection.isBareMode()) return writeBare(connection);
    return writeBottle(connection);
  }

  // This class will serialize ROS style or YARP style depending on protocol.
  // If you need to force a serialization style, use one of these classes:
  typedef yarp::os::idl::BareStyle<geometry_msgs_PointStamped> rosStyle;
  typedef yarp::os::idl::BottleStyle<geometry_msgs_PointStamped> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "# This represents a Point with reference coordinate frame and timestamp\n\
Header header\n\
Point point\n\
\n================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  // Name the class, ROS will need this
  yarp::os::Type getType() {
    yarp::os::Type typ = yarp::os::Type::byName("geometry_msgs/PointStamped","geometry_msgs/PointStamped");
    typ.addProperty("md5sum",yarp::os::Value("c63aecb41bfdfd6b7e1fac37c7cbe7bf"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif
