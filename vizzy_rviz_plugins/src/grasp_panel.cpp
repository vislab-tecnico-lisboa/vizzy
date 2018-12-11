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
#include "../include/vizzy_rviz_plugins/grasp_panel.hpp"


namespace vizzy_rviz_plugins {

GraspPanel::GraspPanel(QWidget *parent)
  : rviz::Panel(parent)
{




  //Configure the push buttons for the handshakes
  go_to_goal = new QPushButton("Go to goal!", this);
  home_button = new QPushButton("Home position", this);
  grab_button = new QPushButton("Grab", this);
  release_button = new QPushButton("Release", this);
  putinbox_button = new QPushButton("Put in box", this);
  giveaway_button = new QPushButton("Give away", this);


  // Next we lay out the "action" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout();
  topic_layout->addWidget( new QLabel( "Action client:" ));
  output_topic_editor_ = new QLineEdit;
  output_topic_editor_->setText("/vizzy/left_arm_cartesian_controller/cartesian_action");
  topic_layout->addWidget(output_topic_editor_ );

  //Initialize action client
  ac = std::make_shared<cartesian_client>(output_topic_editor_->text().toStdString(), true);


  QVBoxLayout* gestures_layout= new QVBoxLayout();
  gestures_layout->addWidget(go_to_goal);
  gestures_layout->addWidget(home_button);
  gestures_layout->addWidget(grab_button);
  gestures_layout->addWidget(release_button);
  gestures_layout->addWidget(putinbox_button);
  gestures_layout->addWidget(giveaway_button);


  QVBoxLayout* panel_layout = new QVBoxLayout();
  panel_layout->addLayout(topic_layout);
  panel_layout->addLayout(gestures_layout);

  setLayout( panel_layout );

  //Connect objects with signals
  connect(home_button, SIGNAL (released()), this, SLOT(home()));
  connect(grab_button, SIGNAL (released()), this, SLOT (grab()));
  connect(release_button, SIGNAL (released()), this, SLOT (release()));
  connect(putinbox_button, SIGNAL (released()), this, SLOT (putinbox()));
  connect(giveaway_button, SIGNAL (released()), this, SLOT (giveaway()));
  connect(go_to_goal, SIGNAL (released()), this, SLOT (gotoGoal()));
  connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));

  updateTopic();
}

void GraspPanel::gotoGoal()
{
  vizzy_msgs::CartesianGoal goal;
  goal.type = goal.CARTESIAN;
  goal.end_effector_pose.position.x = goal_orient_x_;
  goal.end_effector_pose.position.y = goal_orient_y_;
  goal.end_effector_pose.position.z = goal_orient_z_;
  
  goal.end_effector_pose.orientation.x = goal_orient_x_;
  goal.end_effector_pose.orientation.y = goal_orient_y_;
  goal.end_effector_pose.orientation.z = goal_orient_z_;
  goal.end_effector_pose.orientation.w = goal_orient_w_;

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

void GraspPanel::updateTopic()
{
  setTopic( output_topic_editor_->text() );
}

void GraspPanel::setTopic(const QString &new_topic)
{
  // Only take action if the name has changed.
  if( new_topic != output_action_ )
  {
    output_action_ = new_topic;
    // If the topic is the empty string, don't publish anything.
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

void GraspPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_action_ );
}

// Load all configuration data for this panel from the given Config object.
void GraspPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    output_topic_editor_->setText( topic );
    updateTopic();
  }
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vizzy_rviz_plugins::GraspPanel, rviz::Panel)
