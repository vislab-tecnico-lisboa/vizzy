// This is an automatically generated file.
// Generated from this trajectory_msgs_JointTrajectory.msg definition:
//   Header header
//   string[] joint_names
//   JointTrajectoryPoint[] points
// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_trajectory_msgs_JointTrajectory
#define YARPMSG_TYPE_trajectory_msgs_JointTrajectory

#include <string>
#include <vector>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include "TickTime.h"
#include "std_msgs_Header.h"
#include "TickDuration.h"
#include "trajectory_msgs_JointTrajectoryPoint.h"

class trajectory_msgs_JointTrajectory : public yarp::os::idl::WirePortable {
public:
  std_msgs_Header header;
  std::vector<std::string> joint_names;
  std::vector<trajectory_msgs_JointTrajectoryPoint> points;

  trajectory_msgs_JointTrajectory() {
  }

  bool readBare(yarp::os::ConnectionReader& connection) {
    // *** header ***
    if (!header.read(connection)) return false;

    // *** joint_names ***
    int len = connection.expectInt();
    joint_names.resize(len);
    for (int i=0; i<len; i++) {
      int len2 = connection.expectInt();
      joint_names[i].resize(len2);
      if (!connection.expectBlock((char*)joint_names[i].c_str(),len2)) return false;
    }

    // *** points ***
    len = connection.expectInt();
    points.resize(len);
    for (int i=0; i<len; i++) {
      if (!points[i].read(connection)) return false;
    }
    return !connection.isError();
  }

  bool readBottle(yarp::os::ConnectionReader& connection) {
    connection.convertTextMode();
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(3)) return false;

    // *** header ***
    if (!header.read(connection)) return false;

    // *** joint_names ***
    if (connection.expectInt()!=(BOTTLE_TAG_LIST|BOTTLE_TAG_STRING)) return false;
    int len = connection.expectInt();
    joint_names.resize(len);
    for (int i=0; i<len; i++) {
      int len2 = connection.expectInt();
      joint_names[i].resize(len2);
      if (!connection.expectBlock((char*)joint_names[i].c_str(),len2)) return false;
    }

    // *** points ***
    if (connection.expectInt()!=BOTTLE_TAG_LIST) return false;
    len = connection.expectInt();
    points.resize(len);
    for (int i=0; i<len; i++) {
      if (!points[i].read(connection)) return false;
    }
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::read;
  bool read(yarp::os::ConnectionReader& connection) {
    if (connection.isBareMode()) return readBare(connection);
    return readBottle(connection);
  }

  bool writeBare(yarp::os::ConnectionWriter& connection) {
    // *** header ***
    if (!header.write(connection)) return false;

    // *** joint_names ***
    connection.appendInt(joint_names.size());
    for (size_t i=0; i<joint_names.size(); i++) {
      connection.appendInt(joint_names[i].length());
      connection.appendExternalBlock((char*)joint_names[i].c_str(),joint_names[i].length());
    }

    // *** points ***
    connection.appendInt(points.size());
    for (size_t i=0; i<points.size(); i++) {
      if (!points[i].write(connection)) return false;
    }
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(3);

    // *** header ***
    if (!header.write(connection)) return false;

    // *** joint_names ***
    connection.appendInt(BOTTLE_TAG_LIST|BOTTLE_TAG_STRING);
    connection.appendInt(joint_names.size());
    for (size_t i=0; i<joint_names.size(); i++) {
      connection.appendInt(joint_names[i].length());
      connection.appendExternalBlock((char*)joint_names[i].c_str(),joint_names[i].length());
    }

    // *** points ***
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(points.size());
    for (size_t i=0; i<points.size(); i++) {
      if (!points[i].write(connection)) return false;
    }
    connection.convertTextMode();
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::write;
  bool write(yarp::os::ConnectionWriter& connection) {
    if (connection.isBareMode()) return writeBare(connection);
    return writeBottle(connection);
  }

  // This class will serialize ROS style or YARP style depending on protocol.
  // If you need to force a serialization style, use one of these classes:
  typedef yarp::os::idl::BareStyle<trajectory_msgs_JointTrajectory> rosStyle;
  typedef yarp::os::idl::BottleStyle<trajectory_msgs_JointTrajectory> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "Header header\n\
string[] joint_names\n\
JointTrajectoryPoint[] points\n================================================================================\n\
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
MSG: trajectory_msgs/JointTrajectoryPoint\n\
# Each trajectory point specifies either positions[, velocities[, accelerations]]\n\
# or positions[, effort] for the trajectory to be executed.\n\
# All specified values are in the same order as the joint names in JointTrajectory.msg\n\
\n\
float64[] positions\n\
float64[] velocities\n\
float64[] accelerations\n\
float64[] effort\n\
duration time_from_start\n\
";
  }

  // Name the class, ROS will need this
  yarp::os::Type getType() {
    yarp::os::Type typ = yarp::os::Type::byName("trajectory_msgs/JointTrajectory","trajectory_msgs/JointTrajectory");
    typ.addProperty("md5sum",yarp::os::Value("65b4f94a94d1ed67169da35a02f33d3f"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif
