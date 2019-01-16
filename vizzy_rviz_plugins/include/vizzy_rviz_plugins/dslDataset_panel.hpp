/*
Pedro Vicente
based on João Avelino's code
December, 2018
*/

#ifndef dslDataset_PANEL_H
#define dslDataset_PANEL_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <fstream>
#include <rviz/panel.h>
#include <rviz/properties/ros_topic_property.h>
#include <actionlib/client/simple_action_client.h>
#include <QWidget>
#include <QPushButton>
#include <QGroupBox>
#include <QLabel>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <memory.h>
#include <vizzy_msgs/CartesianAction.h>
#include <vizzy_msgs/CartesianActionGoal.h>
#include <vizzy_msgs/CartesianActionFeedback.h>
#include <vizzy_msgs/CartesianActionResult.h>
#include <vizzy_msgs/DslDataset.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


typedef actionlib::SimpleActionClient<vizzy_msgs::CartesianAction> cartesian_client;

class QLineEdit;

namespace vizzy_rviz_plugins{


class dslDatasetPanel: public rviz::Panel
{
  Q_OBJECT
  public:
  dslDatasetPanel(QWidget* parent = 0);

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
   

  // The ROS publisher for the action command
  ros::NodeHandle nh_;
  ros::Subscriber goal_sub_;

  // ROS publisher to save dataset information
  ros::Publisher infoPub;
  std::shared_ptr<cartesian_client> ac;

  //Combo box to choose left or right arm
  QComboBox *arm_combobox_;
  QLabel *arm_label_;
  int selectedArm;



   
private:
  //Push buttons for grasp commands
  QPushButton *home_button;
  QPushButton *action_button;
  QPushButton *recording_button;
  QPushButton *go_to_goal;
  QPushButton *dump_button;

  QPushButton *do_all_button;

  //Controls group
  void createControls(const QString &title);
  QGroupBox *controlsGroup_;

  QLabel *task_vel_label_;
  QLabel *object_label_;
  QLabel *location_label_;
  QLabel *repetition_label_;
  QLabel *time_label_;
  QLabel *trial_label_;

  QDoubleSpinBox *task_vel_spin_;
  QSpinBox *object_spin_;
  QLineEdit *object_name_editor_;
  QSpinBox *location_spin_;
  QSpinBox *repetition_spin_;
  QDoubleSpinBox *time_spin_;
  QSpinBox *trial_spin_;

  //Current Goal
  double goal_pos_x_;
  double goal_pos_y_;
  double goal_pos_z_;
  double goal_orient_x_;
  double goal_orient_y_;
  double goal_orient_z_;
  double goal_orient_w_;

  float linearVelocity_x_;
  float linearVelocity_y_;
  float linearVelocity_z_;
  double duration_traj_;

  // dataset Parameters
  int location_;
  int object_;
  std::vector<std::string> object_names_;
  std::string current_object_name_;
  int repetition_number_;
  int trial_;

  // CSV file
  std::ofstream csvfile;
  double goal_pos_x_offset_ = 0;
  double goal_pos_y_offset_ = 0;
  double goal_pos_z_offset_ = 0;
  double goal_orient_x_offset_ = 0;
  double goal_orient_y_offset_ = 0;
  double goal_orient_z_offset_ = 0;
  double goal_orient_w_offset_ = 0;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

public Q_SLOTS:

  void setAction( const QString& action );
  void setTopic( const QString& topic );
  void home();
  void recording();
  void action();
  void gotoGoal();
  void doAll();
  void updateTaskVel();
  void updateLocation();
  void updateRepetition();
  void updateTime();
  void updateTrial();
  void updateObject();
  void updateObjectName();
  void dumpParameters();
  void initializeParameters();
  void updateArm();
  
protected Q_SLOTS:

  void updateAction();
  void updateTopic();
  void freezeUnfreeze();

};


}


#endif // dslDataset_PANEL_H
