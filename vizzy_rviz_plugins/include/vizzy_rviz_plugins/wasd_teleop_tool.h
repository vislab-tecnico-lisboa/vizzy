#ifndef WASD_TELEOP_TOOL_H
#define WASD_TELEOP_TOOL_H


#include <QIcon>
#include <QMessageBox>
#include <QApplication>

#include <QKeyEvent>

#include <ros/ros.h>

#include <QTimer>
#include <rviz/tool.h>

#include <geometry_msgs/Twist.h>

#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/float_property.h>

#include <actionlib_msgs/GoalID.h>

namespace vizzy_rviz_plugins {


class WasdTeleopTool: public rviz::Tool
{
Q_OBJECT


public:
  WasdTeleopTool();
  ~WasdTeleopTool();
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  void setVel( float linear_velocity_, float angular_velocity_ );



protected:

  void setTopic( const QString& topic );
  void setActionCancelTopic(const QString& new_topic);

  ros::NodeHandle nh_;
  ros::Publisher velocity_publisher_;
  ros::Publisher goal_cancel_publisher_;

  bool boosted_lin_;
  bool boosted_ang_;

  float linear_velocity_;
  float angular_velocity_;

  float lin_step_ = 2;
  float ang_step_ = M_PI/4;

  QString output_topic_;
  rviz::RosTopicProperty *topic_property_;

  rviz::RosTopicProperty *move_base_cancel_topic_property_;


  rviz::FloatProperty *max_lin_property_;
  rviz::FloatProperty *max_ang_property_;

  QTimer* output_timer;

  bool eventFilter(QObject* obj, QEvent* event);


protected Q_SLOTS:

  void maxLinearVelUpdate();

  void maxAngularVelUpdate();

  // sendvel() publishes the current velocity values to a ROS
  // topic.  Internally this is connected to a timer which calls it 10
  // times per second.
  void sendVel();

  // updateTopic() reads the topic name from the RosTopicProperty and calls
  // setTopic() with the result.
  void updateTopic();

  //Same but to cancel any move_base goal
  void updateActionCancelTopic();
};

}

#endif // WASD_TELEOP_TOOL_H
