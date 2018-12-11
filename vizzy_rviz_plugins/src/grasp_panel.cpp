/*
João Avelino
December, 2018
*/

#include <stdio.h>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <math.h> 
#include "../include/vizzy_rviz_plugins/grasp_panel.hpp"


namespace vizzy_rviz_plugins {

GraspPanel::GraspPanel(QWidget *parent)
  : rviz::Panel(parent), tfBuffer(), tfListener(tfBuffer)
{

  //Configure the push buttons for the handshakes
  go_to_goal = new QPushButton("Go to goal!", this);
  home_button = new QPushButton("Home position", this);
  grab_button = new QPushButton("Grab", this);
  release_button = new QPushButton("Release", this);
  putinbox_button = new QPushButton("Put in box", this);
  giveaway_button = new QPushButton("Give away", this);

  freeze_goal_ = false;
  freeze_goal_button_ = new QPushButton("Freeze goal!", this);


  // Next we lay out the "action" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* action_layout = new QHBoxLayout();
  action_layout->addWidget( new QLabel( "Action client:" ));
  output_action_editor_ = new QLineEdit;
  output_action_editor_->setText("/vizzy/left_arm_cartesian_controller/cartesian_action");
  action_layout->addWidget(output_action_editor_ );

  //Initialize action client
  ac = std::make_shared<cartesian_client>(output_action_editor_->text().toStdString(), true);

  // Next we lay out the "topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout();
  topic_layout->addWidget( new QLabel( "Input goal topic:" ));
  input_topic_editor_ = new QLineEdit;
  input_topic_editor_->setText("/left_hand_goal");
  topic_layout->addWidget(input_topic_editor_ );


  //Initialize goal action to update the goal from other nodes (example: ball tracker)
  goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(input_topic_editor_->text().toStdString(), 1, &GraspPanel::poseCallback, this);
  goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/grasp_goal_vis", 1);

  QVBoxLayout* gestures_layout= new QVBoxLayout();
  gestures_layout->addWidget(go_to_goal);
  gestures_layout->addWidget(home_button);
  gestures_layout->addWidget(grab_button);
  gestures_layout->addWidget(release_button);
  gestures_layout->addWidget(putinbox_button);
  gestures_layout->addWidget(giveaway_button);

  QVBoxLayout* controls_out_layout= new QVBoxLayout();

  QGridLayout *controls_layout = new QGridLayout;
  controlsGroup_ = new QGroupBox("End effector controls (current goal)");
  x_label_ = new QLabel(tr("x (cm):"));
  y_label_ = new QLabel(tr("y (cm):"));
  z_label_ = new QLabel(tr("z (cm):"));

  x_spin_ = new QSpinBox();
  x_spin_->setRange(-500, 500);
  x_spin_->setSingleStep(1);

  y_spin_ = new QSpinBox();
  y_spin_->setRange(-500, 500);
  y_spin_->setSingleStep(1);

  z_spin_ = new QSpinBox();
  z_spin_->setRange(-500, 500);
  z_spin_->setSingleStep(1);


  controls_layout->addWidget(x_label_, 0, 0);
  controls_layout->addWidget(x_spin_, 0, 1);
  controls_layout->addWidget(y_label_, 1, 0);
  controls_layout->addWidget(y_spin_, 1, 1);
  controls_layout->addWidget(z_label_, 2, 0);
  controls_layout->addWidget(z_spin_, 2, 1);

  controls_out_layout->addLayout(controls_layout);
  controls_out_layout->addWidget(freeze_goal_button_);
  controlsGroup_->setLayout(controls_out_layout);



  QVBoxLayout* panel_layout = new QVBoxLayout();
  panel_layout->addWidget(controlsGroup_);
  panel_layout->addLayout(gestures_layout);
  panel_layout->addLayout(action_layout);
  panel_layout->addLayout(topic_layout);



  setLayout( panel_layout );

  //Connect objects with signals
  connect(home_button, SIGNAL (released()), this, SLOT(home()));
  connect(grab_button, SIGNAL (released()), this, SLOT (grab()));
  connect(release_button, SIGNAL (released()), this, SLOT (release()));
  connect(putinbox_button, SIGNAL (released()), this, SLOT (putinbox()));
  connect(giveaway_button, SIGNAL (released()), this, SLOT (giveaway()));
  connect(go_to_goal, SIGNAL (released()), this, SLOT (gotoGoal()));
  connect( output_action_editor_, SIGNAL( editingFinished() ), this, SLOT( updateAction() ));
  connect( input_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
  
  connect( x_spin_, SIGNAL( valueChanged(int) ), this, SLOT( updateGoalX() ));
  connect( y_spin_, SIGNAL( valueChanged(int) ), this, SLOT( updateGoalY() ));
  connect( z_spin_, SIGNAL( valueChanged(int) ), this, SLOT( updateGoalZ() ));

  connect(freeze_goal_button_, SIGNAL (released()), this, SLOT (freezeUnfreeze()));


  updateAction();
  updateTopic();
}


void GraspPanel::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  
  if(freeze_goal_)
    return;

  geometry_msgs::TransformStamped transformStamped;
  try{
  transformStamped = tfBuffer.lookupTransform("base_link", msg->header.frame_id,
                              ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  geometry_msgs::PoseStamped onBase;

  tf2::doTransform(*msg, onBase, transformStamped);


  goal_pos_x_ = onBase.pose.position.x;
  goal_pos_y_ = onBase.pose.position.y;
  goal_pos_z_ = onBase.pose.position.z;

  goal_orient_x_ = onBase.pose.orientation.x;
  goal_orient_y_ = onBase.pose.orientation.y;
  goal_orient_z_ = onBase.pose.orientation.z;


  x_spin_->setValue(goal_pos_x_*100);
  y_spin_->setValue(goal_pos_y_*100);
  z_spin_->setValue(goal_pos_z_*100);

  geometry_msgs::PoseStamped pose_viz;

  pose_viz.header.frame_id="base_link";

  pose_viz.pose.position.x = goal_pos_x_+goal_pos_x_offset_;
  pose_viz.pose.position.y = goal_pos_y_+goal_pos_y_offset_;
  pose_viz.pose.position.z = goal_pos_z_+goal_pos_z_offset_;

  double o_x = goal_orient_x_+goal_orient_x_offset_;
  double o_y = goal_orient_y_+goal_orient_y_offset_;
  double o_z = goal_orient_z_+goal_orient_z_offset_;
  pose_viz.pose.orientation.x = o_x;
  pose_viz.pose.orientation.y = o_y;
  pose_viz.pose.orientation.z = o_z; 
  pose_viz.pose.orientation.w = std::sqrt(1.0-(o_x*o_x+o_y*o_y+o_z*o_z));

  goal_pub_.publish(pose_viz);

}

void GraspPanel::gotoGoal()
{
  vizzy_msgs::CartesianGoal goal;
  goal.type = goal.CARTESIAN;
  goal.end_effector_pose.position.x = goal_pos_x_+goal_pos_x_offset_;
  goal.end_effector_pose.position.y = goal_pos_y_+goal_pos_y_offset_;
  goal.end_effector_pose.position.z = goal_pos_z_+goal_pos_z_offset_;
  

  double o_x = goal_orient_x_+goal_orient_x_offset_;
  double o_y = goal_orient_y_+goal_orient_y_offset_;
  double o_z = goal_orient_z_+goal_orient_z_offset_;
  goal.end_effector_pose.orientation.x = o_x;
  goal.end_effector_pose.orientation.y = o_y;
  goal.end_effector_pose.orientation.z = o_z; 
  goal.end_effector_pose.orientation.w = std::sqrt(1.0-(o_x*o_x+o_y*o_y+o_z*o_z));

  ac->sendGoal(goal);
}

void GraspPanel::home()
{
  vizzy_msgs::CartesianGoal goal;
  goal.type = goal.HOME;
  ac->sendGoal(goal);
}

void GraspPanel::grab()
{
  vizzy_msgs::CartesianGoal goal;
  goal.type = goal.GRAB;
  ac->sendGoal(goal);
}

void GraspPanel::release()
{
  vizzy_msgs::CartesianGoal goal;
  goal.type = goal.RELEASE;
  ac->sendGoal(goal);
}

void GraspPanel::putinbox()
{


}

void GraspPanel::giveaway()
{
}

void GraspPanel::updateAction()
{
  setAction( output_action_editor_->text() );
}

void GraspPanel::setAction(const QString &new_action)
{
  // Only take action if the name has changed.
  if( new_action != output_action_ )
  {
    output_action_ = new_action;
    // If the action is the empty string, don't publish anything.
    if( output_action_ == "" )
    {
      ac->cancelAllGoals();
      ac = std::make_shared<cartesian_client>("null", true);
    }
    else
    {
      ac->cancelAllGoals();
      ac = std::make_shared<cartesian_client>(output_action_.toStdString(), true);      
    }

    Q_EMIT configChanged();
  }
}

void GraspPanel::updateTopic()
{
  setTopic( input_topic_editor_->text() );
}

void GraspPanel::setTopic(const QString &new_topic)
{
  // Only take topic if the name has changed.
  if( new_topic != input_topic_ )
  {
    input_topic_ = new_topic;
    // If the action is the empty string, don't subscribe anything.
    if( input_topic_ == "" )
    {
      goal_sub_.shutdown();
    }
    else
    {
      goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(input_topic_.toStdString(), 1, &GraspPanel::poseCallback, this);
    }

    Q_EMIT configChanged();
  }
}

void GraspPanel::freezeUnfreeze()
{
  if(freeze_goal_)
  {
    freeze_goal_ = false;
    freeze_goal_button_->setText("Freeze goal!");
  }
  else{
    freeze_goal_ = true;
    freeze_goal_button_->setText("Unfreeze goal!");
  }

}


void GraspPanel::updateGoalX()
{
  goal_pos_x_ = (double) (x_spin_->value())/100.0;
  geometry_msgs::PoseStamped pose_viz;

  pose_viz.header.frame_id="base_link";

  pose_viz.pose.position.x = goal_pos_x_+goal_pos_x_offset_;
  pose_viz.pose.position.y = goal_pos_y_+goal_pos_y_offset_;
  pose_viz.pose.position.z = goal_pos_z_+goal_pos_z_offset_;

  double o_x = goal_orient_x_+goal_orient_x_offset_;
  double o_y = goal_orient_y_+goal_orient_y_offset_;
  double o_z = goal_orient_z_+goal_orient_z_offset_;
  pose_viz.pose.orientation.x = o_x;
  pose_viz.pose.orientation.y = o_y;
  pose_viz.pose.orientation.z = o_z; 
  pose_viz.pose.orientation.w = std::sqrt(1.0-(o_x*o_x+o_y*o_y+o_z*o_z));

  goal_pub_.publish(pose_viz);
}

void GraspPanel::updateGoalY()
{
  geometry_msgs::PoseStamped pose_viz;

  pose_viz.header.frame_id="base_link";

  pose_viz.pose.position.x = goal_pos_x_+goal_pos_x_offset_;
  pose_viz.pose.position.y = goal_pos_y_+goal_pos_y_offset_;
  pose_viz.pose.position.z = goal_pos_z_+goal_pos_z_offset_;

  double o_x = goal_orient_x_+goal_orient_x_offset_;
  double o_y = goal_orient_y_+goal_orient_y_offset_;
  double o_z = goal_orient_z_+goal_orient_z_offset_;
  pose_viz.pose.orientation.x = o_x;
  pose_viz.pose.orientation.y = o_y;
  pose_viz.pose.orientation.z = o_z; 
  pose_viz.pose.orientation.w = std::sqrt(1.0-(o_x*o_x+o_y*o_y+o_z*o_z));

  goal_pub_.publish(pose_viz);
  goal_pos_y_ = (double) (y_spin_->value())/100.0;
}

void GraspPanel::updateGoalZ()
{
  geometry_msgs::PoseStamped pose_viz;

  pose_viz.header.frame_id="base_link";

  pose_viz.pose.position.x = goal_pos_x_+goal_pos_x_offset_;
  pose_viz.pose.position.y = goal_pos_y_+goal_pos_y_offset_;
  pose_viz.pose.position.z = goal_pos_z_+goal_pos_z_offset_;

  double o_x = goal_orient_x_+goal_orient_x_offset_;
  double o_y = goal_orient_y_+goal_orient_y_offset_;
  double o_z = goal_orient_z_+goal_orient_z_offset_;
  pose_viz.pose.orientation.x = o_x;
  pose_viz.pose.orientation.y = o_y;
  pose_viz.pose.orientation.z = o_z; 
  pose_viz.pose.orientation.w = std::sqrt(1.0-(o_x*o_x+o_y*o_y+o_z*o_z));

  goal_pub_.publish(pose_viz);
  goal_pos_z_ = (double) (z_spin_->value())/100.0;
}


void GraspPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Action", output_action_ );
  config.mapSetValue( "Topic", input_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void GraspPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString action;
  QString topic;
  if( config.mapGetString( "Action", &action ))
  {
    output_action_editor_->setText( action );
    updateAction();
  }

  if( config.mapGetString( "Topic", &topic ))
  {
    input_topic_editor_->setText( topic );
    updateTopic();
  }
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vizzy_rviz_plugins::GraspPanel, rviz::Panel)
