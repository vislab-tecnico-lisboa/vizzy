/*
João Avelino
November, 2017
*/

#include <stdio.h>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <std_msgs/Int16.h>
#include "../include/vizzy_rviz_plugins/hintt_panel.hpp"


namespace vizzy_rviz_plugins {

HINTT_Panel::HINTT_Panel(QWidget *parent)
  : rviz::Panel(parent)
{

  //Configure the push buttons for the handshakes
  home_button_ = new QPushButton("Home position", this);

  hat_start_ = new QPushButton("Hat start", this);
  hat_finish_ = new QPushButton("Hat finish", this);

  letter_start_ = new QPushButton("Letter start", this);
  letter_finish_ = new QPushButton("Letter finish", this);




  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  
  //Left arm
  QHBoxLayout* topic_layout_left = new QHBoxLayout();
  topic_layout_left->addWidget( new QLabel( "Output Topic Left:" ));

  output_topic_editor_left_ = new QLineEdit();
  output_topic_editor_left_->setText("/vizzyArmRoutinesLeft/command"); //CHANGE
  topic_layout_left->addWidget(output_topic_editor_left_);


  //Right arm
  QHBoxLayout* topic_layout_right = new QHBoxLayout();
  topic_layout_right->addWidget( new QLabel( "Output Topic Right:" ));
  output_topic_editor_right_ = new QLineEdit();
  output_topic_editor_right_->setText("/vizzyArmRoutinesRight/command"); //CHANGE
  topic_layout_right->addWidget(output_topic_editor_right_);


  //Delay
  QHBoxLayout* delay_layout = new QHBoxLayout();
  delay_layout->addWidget( new QLabel( "Delay (ms):" ));
  delay_editor_ = new QSpinBox();
  delay_editor_->setMinimum(0);
  delay_editor_->setMaximum(2000);
  delay_editor_->setSingleStep(100);
  delay_editor_->setValue(1000);
  delay_layout->addWidget(delay_editor_);


  QVBoxLayout* gestures_layout= new QVBoxLayout();
  gestures_layout->addWidget(home_button_);
  gestures_layout->addWidget(hat_start_);
  gestures_layout->addWidget(hat_finish_);
  gestures_layout->addWidget(letter_start_);
  gestures_layout->addWidget(letter_finish_);



  QVBoxLayout* panel_layout = new QVBoxLayout();
  panel_layout->addLayout(topic_layout_right);
  panel_layout->addLayout(topic_layout_left);
  panel_layout->addLayout(gestures_layout);
  panel_layout->addLayout(delay_layout);

  setLayout( panel_layout );

  //Connect objects with signals
  connect(home_button_, SIGNAL (released()), this, SLOT(home()));
  connect(hat_start_, SIGNAL (released()), this, SLOT(hatStart()));
  connect(hat_finish_, SIGNAL (released()), this, SLOT(hatFinish()));
  connect(letter_start_, SIGNAL (released()), this, SLOT(letterStart()));
  connect(letter_finish_, SIGNAL (released()), this, SLOT(letterStop()));
  connect(output_topic_editor_left_, SIGNAL( editingFinished() ), this, SLOT( updateLeftTopic() ));
  connect(output_topic_editor_right_, SIGNAL( editingFinished() ), this, SLOT( updateRightTopic() ));

  updateRightTopic();
  updateLeftTopic();
  updateDelay();
}


void HINTT_Panel::command(int cmd)
{
  std_msgs::Int16 command;
  command.data = cmd;
  vizzy_left_arm_publisher_.publish(command);
  ros::Duration(delay_/1000.0).sleep();
  vizzy_right_arm_publisher_.publish(command);
  return;
}

void HINTT_Panel::home()
{
  command(0);
}

void HINTT_Panel::hatStart()
{
  command(6);
}

void HINTT_Panel::hatFinish()
{
  command(7);
}

void HINTT_Panel::letterStart()
{
  command(8);
}

void HINTT_Panel::letterStop()
{
  command(9);
}


void HINTT_Panel::updateRightTopic()
{
  setTopicRight( output_topic_editor_right_->text() );
}

void HINTT_Panel::updateLeftTopic()
{
  setTopicLeft( output_topic_editor_left_->text() );
}

void HINTT_Panel::updateDelay()
{
  delay_ = delay_editor_->value();
}



void HINTT_Panel::setTopicRight(const QString &new_topic)
{
  // Only take action if the name has changed.
  if( new_topic != output_topic_right_ )
  {
    output_topic_right_ = new_topic;
    // If the topic is the empty string, don't publish anything.
    if( output_topic_right_ == "" )
    {
      vizzy_right_arm_publisher_.shutdown();
    }
    else
    {
      vizzy_right_arm_publisher_ = nh_.advertise<std_msgs::Int16>( output_topic_right_.toStdString(), 1 );
    }

    Q_EMIT configChanged();
  }
}

void HINTT_Panel::setTopicLeft(const QString &new_topic)
{
  // Only take action if the name has changed.
  if( new_topic != output_topic_left_ )
  {
    output_topic_left_ = new_topic;
    // If the topic is the empty string, don't publish anything.
    if( output_topic_left_ == "" )
    {
      vizzy_left_arm_publisher_.shutdown();
    }
    else
    {
      vizzy_left_arm_publisher_ = nh_.advertise<std_msgs::Int16>( output_topic_left_.toStdString(), 1 );
    }

    Q_EMIT configChanged();
  }
}

void HINTT_Panel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic right", output_topic_right_ );
  config.mapSetValue( "Topic left", output_topic_left_ );
}

// Load all configuration data for this panel from the given Config object.
void HINTT_Panel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic_right;
  if( config.mapGetString( "Topic right", &topic_right ))
  {
    output_topic_editor_right_->setText( topic_right );
    updateRightTopic();
  }

  QString topic_left;
  if( config.mapGetString( "Topic left", &topic_left ))
  {
    output_topic_editor_left_->setText( topic_left );
    updateLeftTopic();
  }
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vizzy_rviz_plugins::HINTT_Panel, rviz::Panel)
