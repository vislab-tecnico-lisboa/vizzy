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
#include <QDoubleSpinBox>
#include <memory.h>
#include <vizzy_msgs/CartesianAction.h>
#include <vizzy_msgs/CartesianActionGoal.h>
#include <vizzy_msgs/CartesianActionFeedback.h>
#include <vizzy_msgs/CartesianActionResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <interactive_markers/interactive_marker_server.h>


typedef actionlib::SimpleActionClient<vizzy_msgs::CartesianAction> cartesian_client;

class QLineEdit;

using namespace visualization_msgs;

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

  QLabel *x_angle_label_;
  QLabel *y_angle_label_;
  QLabel *z_angle_label_;
  QLabel *w_angle_label_;

  QDoubleSpinBox *x_spin_;
  QDoubleSpinBox *y_spin_;
  QDoubleSpinBox *z_spin_;
  
  QDoubleSpinBox *x_angle_spin_;
  QDoubleSpinBox *y_angle_spin_;
  QDoubleSpinBox *z_angle_spin_;
  QDoubleSpinBox *w_angle_spin_;

  //Current Goal
  double goal_pos_x_;
  double goal_pos_y_;
  double goal_pos_z_;
  double goal_orient_x_;
  double goal_orient_y_;
  double goal_orient_z_;
  double goal_orient_w_;

  double goal_pos_x_offset_ = 0;
  double goal_pos_y_offset_ = 0;
  double goal_pos_z_offset_ = 0;
  double goal_orient_x_offset_ = 0;
  double goal_orient_y_offset_ = 0;
  double goal_orient_z_offset_ = 0;
  double goal_orient_w_offset_ = 0;

  InteractiveMarker int_marker_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );


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
  void updatePoseX();
  void updatePoseY();
  void updatePoseZ();
  void updatePoseW();

  Marker makeEndEffector( InteractiveMarker &msg );
  InteractiveMarkerControl& makeEndEffectorControl( InteractiveMarker &msg );
  void updateMarkerPose();



protected Q_SLOTS:

  void updateAction();
  void updateTopic();
  void freezeUnfreeze();

};


}


#endif // GRASP_PANEL_H
