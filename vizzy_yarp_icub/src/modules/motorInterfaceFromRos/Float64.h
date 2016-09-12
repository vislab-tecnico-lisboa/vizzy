// This is an automatically generated file.
// Generated from this Float64.msg definition:
//   [std_msgs/Float64]:
//   float64 data
// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_Float64
#define YARPMSG_TYPE_Float64

#include <string>
#include <vector>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class Float64 : public yarp::os::idl::WirePortable {
public:
  yarp::os::NetFloat64 data;

  Float64() {
  }

  bool readBare(yarp::os::ConnectionReader& connection) {
    // *** data ***
    data = connection.expectDouble();
    return !connection.isError();
  }

  bool readBottle(yarp::os::ConnectionReader& connection) {
    connection.convertTextMode();
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(1)) return false;

    // *** data ***
    data = reader.expectDouble();
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::read;
  bool read(yarp::os::ConnectionReader& connection) {
    if (connection.isBareMode()) return readBare(connection);
    return readBottle(connection);
  }

  bool writeBare(yarp::os::ConnectionWriter& connection) {
    // *** data ***
    connection.appendDouble(data);
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(1);

    // *** data ***
    connection.appendInt(BOTTLE_TAG_DOUBLE);
    connection.appendDouble((double)data);
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
  typedef yarp::os::idl::BareStyle<Float64> rosStyle;
  typedef yarp::os::idl::BottleStyle<Float64> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "[std_msgs/Float64]:\n\
float64 data";
  }

  // Name the class, ROS will need this
  yarp::os::Type getType() {
    yarp::os::Type typ = yarp::os::Type::byName("Float64","Float64");
    typ.addProperty("md5sum",yarp::os::Value("fdb28210bfa9d7c91146260178d9a584"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif
