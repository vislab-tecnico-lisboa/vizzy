#ifndef HANDSHAKE_PANEL_H
#define HANDSHAKE_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>

class QLineEdit;


namespace vizzy_rviz_plugins{


class GesturesPanel: public rviz::Panel
{
  Q_OBJECT
  public:
  GesturesPanel(Qwidget* parent = 0);

  virtual void load(const rviz::Config &config);
  virtual void save(Config config) const;

public Q_SLOTS:

  void setTopic( const QString& topic );


protected Q_SLOTS:

  void updateTopic();
  // One-line text editor for entering the outgoing ROS topic name.
  rviz::StringProperty *topic_property_;
  // The current name of the output topic.
  QString output_topic_;
  // The ROS publisher for the command velocity.
  ros::Publisher vizzy_arm_publisher;
  ros::NodeHandle nh_;

};


}


#endif // TELEOP_PANEL_H
