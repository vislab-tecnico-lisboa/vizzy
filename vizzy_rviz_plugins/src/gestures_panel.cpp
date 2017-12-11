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
#include "../include/vizzy_rviz_plugins/gestures_panel.hpp"


namespace vizzy_rviz_plugins {

GesturesPanel::GesturesPanel(QWidget *parent)
  : rviz::Panel(parent)
{

  //Configure the push buttons for the handshakes
  home_button = new QPushButton("Home position", this);
  wave_button = new QPushButton("Wave", this);
  stretch_button = new QPushButton("Stretch arm", this);
  handshake_button = new QPushButton("Handshake", this);
  askshake_button = new QPushButton("Ask for handshake", this);

  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout();
  topic_layout->addWidget( new QLabel( "Output Topic:" ));
  output_topic_editor_ = new QLineEdit;
  output_topic_editor_->setText("/vizzyArmRoutines/command");
  topic_layout->addWidget(output_topic_editor_ );

  QVBoxLayout* gestures_layout= new QVBoxLayout();
  gestures_layout->addWidget(home_button);
  gestures_layout->addWidget(wave_button);
  gestures_layout->addWidget(stretch_button);
  gestures_layout->addWidget(handshake_button);
  gestures_layout->addWidget(askshake_button);

  QVBoxLayout* panel_layout = new QVBoxLayout();
  panel_layout->addLayout(topic_layout);
  panel_layout->addLayout(gestures_layout);

  setLayout( panel_layout );

  //Connect objects with signals
  connect(home_button, SIGNAL (released()), this, SLOT(home()));
  connect(wave_button, SIGNAL (released()), this, SLOT (wave()));
  connect(stretch_button, SIGNAL (released()), this, SLOT (stretch()));
  connect(handshake_button, SIGNAL (released()), this, SLOT (handshake()));
  connect(askshake_button, SIGNAL (released()), this, SLOT (askshake()));
  connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));

  updateTopic();
}

void GesturesPanel::home()
{
  std_msgs::Int16 command;
  command.data=0;
  vizzy_arm_publisher.publish(command);
  return;
}

void GesturesPanel::wave()
{
  std_msgs::Int16 command;
  command.data=1;
  vizzy_arm_publisher.publish(command);
  return;
}

void GesturesPanel::stretch()
{
  std_msgs::Int16 command;
  command.data=2;
  vizzy_arm_publisher.publish(command);
}

void GesturesPanel::handshake()
{
  std_msgs::Int16 command;
  command.data=3;
  vizzy_arm_publisher.publish(command);
}

void GesturesPanel::askshake()
{
  std_msgs::Int16 command;
  command.data=4;
  vizzy_arm_publisher.publish(command);  
}

void GesturesPanel::updateTopic()
{
  setTopic( output_topic_editor_->text() );
}

void GesturesPanel::setTopic(const QString &new_topic)
{
  // Only take action if the name has changed.
  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;
    // If the topic is the empty string, don't publish anything.
    if( output_topic_ == "" )
    {
      vizzy_arm_publisher.shutdown();
    }
    else
    {
      vizzy_arm_publisher = nh_.advertise<std_msgs::Int16>( output_topic_.toStdString(), 1 );
    }

    Q_EMIT configChanged();
  }
}

void GesturesPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void GesturesPanel::load( const rviz::Config& config )
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
PLUGINLIB_EXPORT_CLASS(vizzy_rviz_plugins::GesturesPanel, rviz::Panel)
