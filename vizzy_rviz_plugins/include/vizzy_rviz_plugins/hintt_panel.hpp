/*
João Avelino
November, 2017
*/

#ifndef HINTT_PANEL_
#define HINTT_PANEL_

#include <ros/ros.h>
#include <rviz/panel.h>
#include <rviz/properties/ros_topic_property.h>
#include <QWidget>
#include <QPushButton>
#include <QSpinBox>

class QLineEdit;


namespace vizzy_rviz_plugins{


class HINTT_Panel: public rviz::Panel
{
  Q_OBJECT
  public:
  HINTT_Panel(QWidget* parent = 0);

  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;


protected:

   // One-line text editor for entering the outgoing ROS topic name.
   QLineEdit* output_topic_editor_right_;
   QLineEdit* output_topic_editor_left_;
//   QSpinBox* delay_editor_;

   
   // The current name of the output topic.
   QString output_topic_left_;
   QString output_topic_right_;
//   int delay_;
   
  

   // The ROS publisher for the gesture
   ros::Publisher vizzy_left_arm_publisher_;
   ros::Publisher vizzy_right_arm_publisher_;
   ros::NodeHandle nh_;

   void sendCommand(int command);

private:
   
   
   //Push buttons for gestures
   QPushButton *home_button_;

//   QPushButton *hat_start_;
//   QPushButton *hat_finish_;

   QPushButton *letter_start_;
   QPushButton *letter_finish_;
   QPushButton *letter_drop_;
   
   QPushButton *joint_inc_;
   QPushButton *joint_dec_;

public Q_SLOTS:

  void setTopicRight( const QString& topic );
  //void setTopicLeft( const QString& topic );
  void home();

//  void hatStart();
//  void hatFinish();

  void letterStart();
  void letterStop();
  void letterDrop();
  void jointIncrement();
  void jointDecrement();

  void command(int cmd);


protected Q_SLOTS:

  void updateRightTopic();
  //void updateLeftTopic();
//  void updateDelay();


};


}


#endif // HINTT_PANEL_
