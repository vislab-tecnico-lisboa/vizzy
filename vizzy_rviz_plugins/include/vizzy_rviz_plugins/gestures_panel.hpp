/*
João Avelino
November, 2017
*/

#ifndef HANDSHAKE_PANEL_H
#define HANDSHAKE_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <rviz/properties/ros_topic_property.h>
#include <QWidget>
#include <QPushButton>

class QLineEdit;


namespace vizzy_rviz_plugins{


class GesturesPanel: public rviz::Panel
{
  Q_OBJECT
  public:
  GesturesPanel(QWidget* parent = 0);

  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;


protected:

  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit* output_topic_editor_;
   // The current name of the output topic.
   QString output_topic_;
   //Push buttons for gestures

   // The ROS publisher for the gesture
   ros::Publisher vizzy_arm_publisher;
   ros::NodeHandle nh_;

   void sendCommand(int command);

private:
   QPushButton *home_button;
   QPushButton *wave_button;
   QPushButton *stretch_button;
   QPushButton *askshake_button;
   QPushButton *handshake_button;
   QPushButton *handshake_pid_button;

public Q_SLOTS:

  void setTopic( const QString& topic );
  void home();
  void wave();
  void stretch();
  void handshake();
  void askshake();
  void handshake_pid();


protected Q_SLOTS:

  void updateTopic();


};


}


#endif // HANDSHAKE_PANEL_H
