// This is an automatically generated file.
// Generated from this Header.msg definition:
//   [std_msgs/Header]:
//   # Standard metadata for higher-level stamped data types.
//   # This is generally used to communicate timestamped data 
//   # in a particular coordinate frame.
//   # 
//   # sequence ID: consecutively increasing ID 
//   uint32 seq
//   #Two-integer timestamp that is expressed as:
//   # * stamp.secs: seconds (stamp_secs) since epoch
//   # * stamp.nsecs: nanoseconds since stamp_secs
//   # time-handling sugar is provided by the client library
//   time stamp
//   #Frame this data is associated with
//   # 0: no frame
//   # 1: global frame
//   string frame_id
//   
// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_Header
#define YARPMSG_TYPE_Header

#include <string>
#include <vector>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include "TickTime.h"

class Header : public yarp::os::idl::WirePortable {
public:
  yarp::os::NetUint32 seq;
  TickTime stamp;
  std::string frame_id;

  Header() {
  }

  bool readBare(yarp::os::ConnectionReader& connection) {
    // *** seq ***
    seq = connection.expectInt();

    // *** stamp ***
    if (!stamp.read(connection)) return false;

    // *** frame_id ***
    int len = connection.expectInt();
    frame_id.resize(len);
    if (!connection.expectBlock((char*)frame_id.c_str(),len)) return false;
    return !connection.isError();
  }

  bool readBottle(yarp::os::ConnectionReader& connection) {
    connection.convertTextMode();
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(3)) return false;

    // *** seq ***
    seq = reader.expectInt();

    // *** stamp ***
    if (!stamp.read(connection)) return false;

    // *** frame_id ***
    if (!reader.readString(frame_id)) return false;
    return !connection.isError();
  }

  bool read(yarp::os::ConnectionReader& connection) {
    if (connection.isBareMode()) return readBare(connection);
    return readBottle(connection);
  }

  bool writeBare(yarp::os::ConnectionWriter& connection) {
    // *** seq ***
    connection.appendInt(seq);

    // *** stamp ***
    if (!stamp.write(connection)) return false;

    // *** frame_id ***
    connection.appendInt(frame_id.length());
    connection.appendExternalBlock((char*)frame_id.c_str(),frame_id.length());
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(3);

    // *** seq ***
    connection.appendInt(BOTTLE_TAG_INT);
    connection.appendInt((int)seq);

    // *** stamp ***
    if (!stamp.write(connection)) return false;

    // *** frame_id ***
    connection.appendInt(BOTTLE_TAG_STRING);
    connection.appendInt(frame_id.length());
    connection.appendExternalBlock((char*)frame_id.c_str(),frame_id.length());
    connection.convertTextMode();
    return !connection.isError();
  }

  bool write(yarp::os::ConnectionWriter& connection) {
    if (connection.isBareMode()) return writeBare(connection);
    return writeBottle(connection);
  }

  // This class will serialize ROS style or YARP style depending on protocol.
  // If you need to force a serialization style, use one of these classes:
  typedef yarp::os::idl::BareStyle<Header> rosStyle;
  typedef yarp::os::idl::BottleStyle<Header> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "# Standard metadata for higher-level stamped data types.\n\
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
";
  }

  // Name the class, ROS will need this
  yarp::os::Type getType() {
    yarp::os::Type typ = yarp::os::Type::byName("Header","Header");
    typ.addProperty("md5sum",yarp::os::Value("2176decaecbce78abc3b96ef049fabed"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif
