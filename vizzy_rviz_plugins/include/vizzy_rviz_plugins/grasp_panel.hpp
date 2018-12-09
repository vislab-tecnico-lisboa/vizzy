/*
João Avelino
December, 2018
*/

#ifndef GRASP_PANEL_H
#define GRASP_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <rviz/properties/ros_topic_property.h>
#include <QWidget>
#include <QPushButton>

class QLineEdit;


namespace vizzy_rviz_plugins{


class GraspPanel: public rviz::Panel
{
  Q_OBJECT
  public:
  GraspPanel(QWidget* parent = 0);

  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;


protected:

  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit* output_topic_editor_;
   // The current name of the output topic.
   QString output_topic_;
   //Push buttons for grasp commands

   // The ROS publisher for the grasp command
   ros::Publisher vizzy_arm_publisher;
   ros::NodeHandle nh_;

   void sendCommand(int command);

private:
   QPushButton *home_button;
   QPushButton *grab_button;
   QPushButton *release_button;
   QPushButton *putinbox_button;
   QPushButton *giveaway_button;

public Q_SLOTS:

  void setTopic( const QString& topic );
  void home();
  void grab();
  void release();
  void putinbox();
  void giveaway();

protected Q_SLOTS:

  void updateTopic();


};


}


#endif // GRASP_PANEL_H
