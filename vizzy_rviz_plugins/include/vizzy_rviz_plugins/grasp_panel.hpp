/*
João Avelino
December, 2018
*/

#ifndef GRASP_PANEL_H
#define GRASP_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <rviz/properties/ros_topic_property.h>
#include <actionlib/client/simple_action_client.h>
#include <QWidget>
#include <QPushButton>
#include <memory.h>
#include <vizzy_msgs/CartesianAction.h>
#include <vizzy_msgs/CartesianActionGoal.h>
#include <vizzy_msgs/CartesianActionFeedback.h>
#include <vizzy_msgs/CartesianActionResult.h>

typedef actionlib::SimpleActionClient<vizzy_msgs::CartesianAction> cartesian_client;

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
   QString output_action_;
   //Push buttons for grasp commands

   // The ROS publisher for the grasp command
    ros::NodeHandle nh_;
    std::shared_ptr<cartesian_client> ac;

   
private:
   QPushButton *home_button;
   QPushButton *grab_button;
   QPushButton *release_button;
   QPushButton *putinbox_button;
   QPushButton *giveaway_button;
   QPushButton *go_to_goal;


   //Current Goal
   double goal_pos_x_;
   double goal_pos_y_;
   double goal_pos_z_;
   double goal_orient_x_;
   double goal_orient_y_;
   double goal_orient_z_;
   double goal_orient_w_;




public Q_SLOTS:

  void setTopic( const QString& topic );
  void home();
  void grab();
  void release();
  void putinbox();
  void giveaway();
  void gotoGoal();

protected Q_SLOTS:

  void updateTopic();


};


}


#endif // GRASP_PANEL_H
