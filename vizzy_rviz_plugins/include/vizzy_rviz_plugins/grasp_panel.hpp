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
#include <QGroupBox>
#include <QLabel>
#include <QSpinBox>
#include <memory.h>
#include <vizzy_msgs/CartesianAction.h>
#include <vizzy_msgs/CartesianActionGoal.h>
#include <vizzy_msgs/CartesianActionFeedback.h>
#include <vizzy_msgs/CartesianActionResult.h>
#include <geometry_msgs/PoseStamped.h>

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

  // One-line text editor for entering the outgoing ROS action name.
   QLineEdit* output_action_editor_;
   // The current name of the output action.
   QString output_action_;

  // One-line text editor for entering the outgoing ROS topic name.
   QLineEdit* input_topic_editor_;
   // The current name of the output topic.
   QString input_topic_;
   

   // The ROS publisher for the grasp command
    ros::NodeHandle nh_;
    ros::Subscriber goal_sub_;

    //For visualization
    ros::Publisher goal_pub_;

    std::shared_ptr<cartesian_client> ac;



   
private:
  //Push buttons for grasp commands
  QPushButton *home_button;
  QPushButton *grab_button;
  QPushButton *release_button;
  QPushButton *putinbox_button;
  QPushButton *giveaway_button;
  QPushButton *go_to_goal;
  QPushButton *freeze_goal_button_;

  //Controls group
  bool freeze_goal_ = false;
  void createControls(const QString &title);
  QGroupBox *controlsGroup_;
  QLabel *x_label_;
  QLabel *y_label_;
  QLabel *z_label_;

  QSpinBox *x_spin_;
  QSpinBox *y_spin_;
  QSpinBox *z_spin_;
  
  //Current Goal
  double goal_pos_x_;
  double goal_pos_y_;
  double goal_pos_z_;
  double goal_orient_x_;
  double goal_orient_y_;
  double goal_orient_z_;

  double goal_pos_x_offset_ = 0;
  double goal_pos_y_offset_ = 0;
  double goal_pos_z_offset_ = 0;
  double goal_orient_x_offset_ = 0;
  double goal_orient_y_offset_ = 0;
  double goal_orient_z_offset_ = 0; 

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);



public Q_SLOTS:

  void setAction( const QString& action );
  void setTopic( const QString& topic );
  void home();
  void grab();
  void release();
  void putinbox();
  void giveaway();
  void gotoGoal();
  void updateGoalX();
  void updateGoalY();
  void updateGoalZ();

protected Q_SLOTS:

  void updateAction();
  void updateTopic();
  void freezeUnfreeze();

};


}


#endif // GRASP_PANEL_H
