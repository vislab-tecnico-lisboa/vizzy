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
  QLineEdit* output_head_topic_editor_;
   // The current name of the output topic.
   QString output_topic_;
   QString output_head_topic_;
   //Push buttons for gestures

   // The ROS publisher for the gesture
   ros::Publisher vizzy_arm_publisher1;
   ros::Publisher vizzy_arm_publisher2;
   ros::Publisher vizzy_head_publisher;
   ros::NodeHandle nh_;
   bool bothArmsFound = false;

   void sendCommand(int command);
   void sendBothArms(int cmd_right, int cmd_left);

private:
   QPushButton *home_button;
   QPushButton *wave_button;
   QPushButton *stretch_button;
   QPushButton *askshake_button;
   QPushButton *handshake_button;
   QPushButton *handshake_pid_button;
   QPushButton *arm_down_button;
   QPushButton *happy_emotionless_button;
   QPushButton *happy_emotive_button;
   QPushButton *sad_button;
   QPushButton *angry_button;
   QPushButton *fear_button;
   QPushButton *surprise_button;
   QPushButton *stretch_open_button;
   QPushButton *surprise_open_button;
   QPushButton *head_dip_button;

public Q_SLOTS:

  void setTopic( const QString& topic );
  void setHeadTopic( const QString& topic );
  void home();
  void wave();
  void stretch();
  void handshake();
  void askshake();
  void handshake_pid();
  void head_dip();


  //New expressive gestures from Joaquim Rocha, Joao Saramago, and Rodrigo Santos
  void arm_down();
  void happy_emotionless();
  void happy_emotive();
  void sad();
  void angry();
  void fear();
  void surprise();
  void stretch_open();
  void surprise_open();


protected Q_SLOTS:

  void updateTopic();
  void updateHeadTopic();


};


}


#endif // HANDSHAKE_PANEL_H
