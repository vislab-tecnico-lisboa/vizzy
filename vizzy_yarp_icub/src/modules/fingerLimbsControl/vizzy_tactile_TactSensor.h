// This is an automatically generated file.
// Generated from this vizzy_tactile_TactSensor.msg definition:
//   string frame_id
//   geometry_msgs/Vector3 force
//   geometry_msgs/Vector3 displacement
//   
// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_vizzy_tactile_TactSensor
#define YARPMSG_TYPE_vizzy_tactile_TactSensor

#include <string>
#include <vector>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include "TickTime.h"
#include "std_msgs_Header.h"
#include "geometry_msgs_Vector3.h"

class vizzy_tactile_TactSensor : public yarp::os::idl::WirePortable {
public:
  std::string frame_id;
  geometry_msgs_Vector3 force;
  geometry_msgs_Vector3 displacement;

  vizzy_tactile_TactSensor() :
    frame_id(""),
    force(),
    displacement()
  {
  }

  void clear() {
    // *** frame_id ***
    frame_id = "";

    // *** force ***
    force.clear();

    // *** displacement ***
    displacement.clear();
  }

  bool readBare(yarp::os::ConnectionReader& connection) YARP_OVERRIDE {
    // *** frame_id ***
    int len = connection.expectInt();
    frame_id.resize(len);
    if (!connection.expectBlock((char*)frame_id.c_str(),len)) return false;

    // *** force ***
    if (!force.read(connection)) return false;

    // *** displacement ***
    if (!displacement.read(connection)) return false;
    return !connection.isError();
  }

  bool readBottle(yarp::os::ConnectionReader& connection) YARP_OVERRIDE {
    connection.convertTextMode();
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(3)) return false;

    // *** frame_id ***
    if (!reader.readString(frame_id)) return false;

    // *** force ***
    if (!force.read(connection)) return false;

    // *** displacement ***
    if (!displacement.read(connection)) return false;
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::read;
  bool read(yarp::os::ConnectionReader& connection) YARP_OVERRIDE {
    if (connection.isBareMode()) return readBare(connection);
    return readBottle(connection);
  }

  bool writeBare(yarp::os::ConnectionWriter& connection) YARP_OVERRIDE {
    // *** frame_id ***
    connection.appendInt(frame_id.length());
    connection.appendExternalBlock((char*)frame_id.c_str(),frame_id.length());

    // *** force ***
    if (!force.write(connection)) return false;

    // *** displacement ***
    if (!displacement.write(connection)) return false;
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) YARP_OVERRIDE {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(3);

    // *** frame_id ***
    connection.appendInt(BOTTLE_TAG_STRING);
    connection.appendInt(frame_id.length());
    connection.appendExternalBlock((char*)frame_id.c_str(),frame_id.length());

    // *** force ***
    if (!force.write(connection)) return false;

    // *** displacement ***
    if (!displacement.write(connection)) return false;
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
  typedef yarp::os::idl::BareStyle<vizzy_tactile_TactSensor> rosStyle;
  typedef yarp::os::idl::BottleStyle<vizzy_tactile_TactSensor> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "string frame_id\n\
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
    yarp::os::Type typ = yarp::os::Type::byName("vizzy_tactile/TactSensor","vizzy_tactile/TactSensor");
    typ.addProperty("md5sum",yarp::os::Value("1c6eb4af01a1ab814514891dca5916ec"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif
