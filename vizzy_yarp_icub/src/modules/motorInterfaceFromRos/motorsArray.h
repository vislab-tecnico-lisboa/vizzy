// This is an automatically generated file.
// Generated from this motorsArray.msg definition:
//   float64[] data
// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_motorsArray
#define YARPMSG_TYPE_motorsArray

#include <string>
#include <vector>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class motorsArray : public yarp::os::idl::WirePortable {
public:
  std::vector<yarp::os::NetFloat64> data;

  motorsArray() :
    data()
  {
  }

  void clear() {
    // *** data ***
    data.clear();
  }

  bool readBare(yarp::os::ConnectionReader& connection) YARP_OVERRIDE {
    // *** data ***
    int len = connection.expectInt();
    data.resize(len);
    if (len > 0 && !connection.expectBlock((char*)&data[0],sizeof(yarp::os::NetFloat64)*len)) return false;
    return !connection.isError();
  }

  bool readBottle(yarp::os::ConnectionReader& connection) YARP_OVERRIDE {
    connection.convertTextMode();
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(1)) return false;

    // *** data ***
    if (connection.expectInt()!=(BOTTLE_TAG_LIST|BOTTLE_TAG_DOUBLE)) return false;
    int len = connection.expectInt();
    data.resize(len);
    for (int i=0; i<len; i++) {
      data[i] = (yarp::os::NetFloat64)connection.expectDouble();
    }
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::read;
  bool read(yarp::os::ConnectionReader& connection) YARP_OVERRIDE {
    if (connection.isBareMode()) return readBare(connection);
    return readBottle(connection);
  }

  bool writeBare(yarp::os::ConnectionWriter& connection) YARP_OVERRIDE {
    // *** data ***
    connection.appendInt(data.size());
    if (data.size()>0) {connection.appendExternalBlock((char*)&data[0],sizeof(yarp::os::NetFloat64)*data.size());}
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) YARP_OVERRIDE {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(1);

    // *** data ***
    connection.appendInt(BOTTLE_TAG_LIST|BOTTLE_TAG_DOUBLE);
    connection.appendInt(data.size());
    for (size_t i=0; i<data.size(); i++) {
      connection.appendDouble((double)data[i]);
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
  typedef yarp::os::idl::BareStyle<motorsArray> rosStyle;
  typedef yarp::os::idl::BottleStyle<motorsArray> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "float64[] data";
  }

  // Name the class, ROS will need this
  yarp::os::Type getType() YARP_OVERRIDE {
    yarp::os::Type typ = yarp::os::Type::byName("motorsArray","motorsArray");
    typ.addProperty("md5sum",yarp::os::Value("788898178a3da2c3718461eecda8f714"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif
