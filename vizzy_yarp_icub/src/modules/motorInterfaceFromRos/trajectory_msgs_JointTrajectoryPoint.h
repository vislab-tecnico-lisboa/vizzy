// This is an automatically generated file.
// Generated from this trajectory_msgs_JointTrajectoryPoint.msg definition:
//   # Each trajectory point specifies either positions[, velocities[, accelerations]]
//   # or positions[, effort] for the trajectory to be executed.
//   # All specified values are in the same order as the joint names in JointTrajectory.msg
//   
//   float64[] positions
//   float64[] velocities
//   float64[] accelerations
//   float64[] effort
//   duration time_from_start
//   
// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_trajectory_msgs_JointTrajectoryPoint
#define YARPMSG_TYPE_trajectory_msgs_JointTrajectoryPoint

#include <string>
#include <vector>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include "TickTime.h"
#include "std_msgs_Header.h"
#include "TickDuration.h"

class trajectory_msgs_JointTrajectoryPoint : public yarp::os::idl::WirePortable {
public:
  std::vector<yarp::os::NetFloat64> positions;
  std::vector<yarp::os::NetFloat64> velocities;
  std::vector<yarp::os::NetFloat64> accelerations;
  std::vector<yarp::os::NetFloat64> effort;
  TickDuration time_from_start;

  trajectory_msgs_JointTrajectoryPoint() {
  }

  bool readBare(yarp::os::ConnectionReader& connection) {
    // *** positions ***
    int len = connection.expectInt();
    positions.resize(len);
    if (!connection.expectBlock((char*)&positions[0],sizeof(yarp::os::NetFloat64)*len)) return false;

    // *** velocities ***
    len = connection.expectInt();
    velocities.resize(len);
    if (!connection.expectBlock((char*)&velocities[0],sizeof(yarp::os::NetFloat64)*len)) return false;

    // *** accelerations ***
    len = connection.expectInt();
    accelerations.resize(len);
    if (!connection.expectBlock((char*)&accelerations[0],sizeof(yarp::os::NetFloat64)*len)) return false;

    // *** effort ***
    len = connection.expectInt();
    effort.resize(len);
    if (!connection.expectBlock((char*)&effort[0],sizeof(yarp::os::NetFloat64)*len)) return false;

    // *** time_from_start ***
    if (!time_from_start.read(connection)) return false;
    return !connection.isError();
  }

  bool readBottle(yarp::os::ConnectionReader& connection) {
    connection.convertTextMode();
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(5)) return false;

    // *** positions ***
    if (connection.expectInt()!=(BOTTLE_TAG_LIST|BOTTLE_TAG_DOUBLE)) return false;
    int len = connection.expectInt();
    positions.resize(len);
    for (int i=0; i<len; i++) {
      positions[i] = (yarp::os::NetFloat64)connection.expectDouble();
    }

    // *** velocities ***
    if (connection.expectInt()!=(BOTTLE_TAG_LIST|BOTTLE_TAG_DOUBLE)) return false;
    len = connection.expectInt();
    velocities.resize(len);
    for (int i=0; i<len; i++) {
      velocities[i] = (yarp::os::NetFloat64)connection.expectDouble();
    }

    // *** accelerations ***
    if (connection.expectInt()!=(BOTTLE_TAG_LIST|BOTTLE_TAG_DOUBLE)) return false;
    len = connection.expectInt();
    accelerations.resize(len);
    for (int i=0; i<len; i++) {
      accelerations[i] = (yarp::os::NetFloat64)connection.expectDouble();
    }

    // *** effort ***
    if (connection.expectInt()!=(BOTTLE_TAG_LIST|BOTTLE_TAG_DOUBLE)) return false;
    len = connection.expectInt();
    effort.resize(len);
    for (int i=0; i<len; i++) {
      effort[i] = (yarp::os::NetFloat64)connection.expectDouble();
    }

    // *** time_from_start ***
    if (!time_from_start.read(connection)) return false;
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::read;
  bool read(yarp::os::ConnectionReader& connection) {
    if (connection.isBareMode()) return readBare(connection);
    return readBottle(connection);
  }

  bool writeBare(yarp::os::ConnectionWriter& connection) {
    // *** positions ***
    connection.appendInt(positions.size());
    connection.appendExternalBlock((char*)&positions[0],sizeof(yarp::os::NetFloat64)*positions.size());

    // *** velocities ***
    connection.appendInt(velocities.size());
    connection.appendExternalBlock((char*)&velocities[0],sizeof(yarp::os::NetFloat64)*velocities.size());

    // *** accelerations ***
    connection.appendInt(accelerations.size());
    connection.appendExternalBlock((char*)&accelerations[0],sizeof(yarp::os::NetFloat64)*accelerations.size());

    // *** effort ***
    connection.appendInt(effort.size());
    connection.appendExternalBlock((char*)&effort[0],sizeof(yarp::os::NetFloat64)*effort.size());

    // *** time_from_start ***
    if (!time_from_start.write(connection)) return false;
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(5);

    // *** positions ***
    connection.appendInt(BOTTLE_TAG_LIST|BOTTLE_TAG_DOUBLE);
    connection.appendInt(positions.size());
    for (size_t i=0; i<positions.size(); i++) {
      connection.appendDouble((double)positions[i]);
    }

    // *** velocities ***
    connection.appendInt(BOTTLE_TAG_LIST|BOTTLE_TAG_DOUBLE);
    connection.appendInt(velocities.size());
    for (size_t i=0; i<velocities.size(); i++) {
      connection.appendDouble((double)velocities[i]);
    }

    // *** accelerations ***
    connection.appendInt(BOTTLE_TAG_LIST|BOTTLE_TAG_DOUBLE);
    connection.appendInt(accelerations.size());
    for (size_t i=0; i<accelerations.size(); i++) {
      connection.appendDouble((double)accelerations[i]);
    }

    // *** effort ***
    connection.appendInt(BOTTLE_TAG_LIST|BOTTLE_TAG_DOUBLE);
    connection.appendInt(effort.size());
    for (size_t i=0; i<effort.size(); i++) {
      connection.appendDouble((double)effort[i]);
    }

    // *** time_from_start ***
    if (!time_from_start.write(connection)) return false;
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
  typedef yarp::os::idl::BareStyle<trajectory_msgs_JointTrajectoryPoint> rosStyle;
  typedef yarp::os::idl::BottleStyle<trajectory_msgs_JointTrajectoryPoint> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "# Each trajectory point specifies either positions[, velocities[, accelerations]]\n\
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
    yarp::os::Type typ = yarp::os::Type::byName("trajectory_msgs/JointTrajectoryPoint","trajectory_msgs/JointTrajectoryPoint");
    typ.addProperty("md5sum",yarp::os::Value("f3cd1e1c4d320c79d6985c904ae5dcd3"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif
