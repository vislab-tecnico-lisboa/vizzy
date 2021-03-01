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
#include <QScrollArea>
#include <QWidget>
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
  handshake_button = new QPushButton("Handshake fixed", this);
  askshake_button = new QPushButton("Ask for handshake", this);
  handshake_pid_button = new QPushButton("Handshake PID", this);
  arm_down_button = new QPushButton("Arms down", this);
  happy_emotionless_button = new QPushButton("Happy (emotionless, 2 arms)", this);
  happy_emotive_button = new QPushButton("Happy (emotive, 2 arms)", this);
  sad_button = new QPushButton("Sad (2 arms)", this);
  angry_button = new QPushButton("Angry (2 arms)", this);
  fear_button = new QPushButton("Fear (2 arms)", this);
  surprise_button = new QPushButton("Surprise", this);
  stretch_open_button = new QPushButton("Stretch (Open)", this);
  surprise_open_button = new QPushButton("Surprise (Open)", this);
  head_dip_button = new QPushButton("Head dip", this);




  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout();
  topic_layout->addWidget( new QLabel( "Output Topics (Arms/Head):" ));
  output_topic_editor_ = new QLineEdit;
  output_topic_editor_->setText("/vizzyArmRoutines/right/command");
  topic_layout->addWidget(output_topic_editor_ );
  output_head_topic_editor_ = new QLineEdit;
  output_head_topic_editor_->setText("/head_topic");
  topic_layout->addWidget(output_head_topic_editor_ );



  QWidget *central = new QWidget();
  QScrollArea *scrollArea = new QScrollArea();


  QVBoxLayout* gestures_layout= new QVBoxLayout(central);
  scrollArea->setWidget(central);
  scrollArea->setWidgetResizable(true);

  gestures_layout->addWidget(home_button);
  gestures_layout->addWidget(head_dip_button);
  gestures_layout->addWidget(wave_button);
  gestures_layout->addWidget(stretch_button);
  gestures_layout->addWidget(handshake_button);
  gestures_layout->addWidget(askshake_button);
  gestures_layout->addWidget(handshake_pid_button);

  //New expressive gestures from Joaquim Rocha, Joao Saramago, and Rodrigo Santos
  gestures_layout->addWidget(arm_down_button);
  gestures_layout->addWidget(happy_emotionless_button);
  gestures_layout->addWidget(happy_emotive_button);
  gestures_layout->addWidget(sad_button);
  gestures_layout->addWidget(angry_button);
  gestures_layout->addWidget(fear_button);
  gestures_layout->addWidget(surprise_button);
  gestures_layout->addWidget(stretch_open_button);
  gestures_layout->addWidget(surprise_open_button);

  QVBoxLayout* panel_layout = new QVBoxLayout();
  panel_layout->addLayout(topic_layout);
  panel_layout->addWidget(scrollArea);

  setLayout( panel_layout );

  //Connect objects with signals
  connect(home_button, SIGNAL (released()), this, SLOT(home()));
  connect(head_dip_button, SIGNAL (released()), this, SLOT(head_dip()));
  connect(wave_button, SIGNAL (released()), this, SLOT (wave()));
  connect(stretch_button, SIGNAL (released()), this, SLOT (stretch()));
  connect(handshake_button, SIGNAL (released()), this, SLOT (handshake()));
  connect(handshake_pid_button, SIGNAL (released()), this, SLOT (handshake_pid()));
  connect(output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
  connect(output_head_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateHeadTopic() ));

  connect(arm_down_button, SIGNAL (released()), this, SLOT(arm_down()));
  connect(happy_emotionless_button, SIGNAL (released()), this, SLOT(happy_emotionless()));
  connect(happy_emotive_button, SIGNAL (released()), this, SLOT(happy_emotive()));
  connect(sad_button, SIGNAL (released()), this, SLOT(sad()));
  connect(angry_button, SIGNAL (released()), this, SLOT(angry()));
  connect(fear_button, SIGNAL (released()), this, SLOT(fear()));
  connect(surprise_button, SIGNAL (released()), this, SLOT(surprise()));
  connect(stretch_open_button, SIGNAL (released()), this, SLOT(stretch_open()));
  connect(surprise_open_button, SIGNAL (released()), this, SLOT(surprise_open()));

  updateTopic();
  updateHeadTopic();
}


void GesturesPanel::home()
{
  std_msgs::Int16 command;
  command.data=0;
  vizzy_arm_publisher1.publish(command);
  return;
}

void GesturesPanel::wave()
{
  std_msgs::Int16 command;
  command.data=1;
  vizzy_arm_publisher1.publish(command);
  return;
}

void GesturesPanel::stretch()
{
  std_msgs::Int16 command;
  command.data=2;
  vizzy_arm_publisher1.publish(command);
}

void GesturesPanel::handshake()
{
  std_msgs::Int16 command;
  command.data=3;
  vizzy_arm_publisher1.publish(command);
}

void GesturesPanel::askshake()
{
  std_msgs::Int16 command;
  command.data=4;
  vizzy_arm_publisher1.publish(command);  
}

void GesturesPanel::handshake_pid()
{
  std_msgs::Int16 command;
  command.data=5;
  vizzy_arm_publisher1.publish(command);
}

void GesturesPanel::arm_down()
{

  std_msgs::Int16 command;
  command.data=6;
  vizzy_arm_publisher1.publish(command);

  if(bothArmsFound)
    vizzy_arm_publisher2.publish(command);

}

void GesturesPanel::happy_emotionless()
{

  std_msgs::Int16 command;
  command.data=7;
  vizzy_arm_publisher1.publish(command);

  if(bothArmsFound)
    vizzy_arm_publisher2.publish(command);

}

void GesturesPanel::happy_emotive()
{

  std_msgs::Int16 command;
  command.data=8;
  vizzy_arm_publisher1.publish(command);
  if(bothArmsFound)
    vizzy_arm_publisher2.publish(command);

}

void GesturesPanel::sad()
{

  std_msgs::Int16 command;
  command.data=9;
  vizzy_arm_publisher1.publish(command);
  if(bothArmsFound)
    vizzy_arm_publisher2.publish(command);

}

void GesturesPanel::angry()
{

  std_msgs::Int16 command;
  command.data=10;
  vizzy_arm_publisher1.publish(command);
  if(bothArmsFound)
    vizzy_arm_publisher2.publish(command);
  
}

void GesturesPanel::fear()
{

  if(!bothArmsFound)
  {
    ROS_INFO("Warning 208: Check out Vizzy Arm Gestures topic. I could not find the keyword left or right... So I don't know which arm you are using!");
    return;
  }

  std_msgs::Int16 right_cmd;
  std_msgs::Int16 left_cmd;

  right_cmd.data = 11;
  left_cmd.data = 12;

  size_t index;

  std::string main_arm_topic = vizzy_arm_publisher1.getTopic();

  index = main_arm_topic.find("right", 0);
  if(index == std::string::npos)
  {
    index = main_arm_topic.find("left", 0);
    if(index == std::string::npos)
    {
      ROS_INFO("Warning 228: Check out Vizzy Arm Gestures topic. I could not find the keyword left or right... So I don't know which arm you are using!");
      return;
    }else{
      vizzy_arm_publisher1.publish(left_cmd);
      vizzy_arm_publisher2.publish(right_cmd);
    }
  }else{
    vizzy_arm_publisher1.publish(right_cmd);
    vizzy_arm_publisher2.publish(left_cmd);
  }

  return;
  
}

void GesturesPanel::surprise()
{

  std_msgs::Int16 command;
  command.data=13;
  vizzy_arm_publisher1.publish(command);
  if(bothArmsFound)
    vizzy_arm_publisher2.publish(command);
  
}

void GesturesPanel::stretch_open()
{
  
  std_msgs::Int16 command;
  command.data=14;
  vizzy_arm_publisher1.publish(command);
  if(bothArmsFound)
    vizzy_arm_publisher2.publish(command);
}

void GesturesPanel::surprise_open()
{
  
  std_msgs::Int16 command;
  command.data=15;
  vizzy_arm_publisher1.publish(command);
  if(bothArmsFound)
    vizzy_arm_publisher2.publish(command);

}

void GesturesPanel::head_dip()
{
  
  std_msgs::Int16 command;
  command.data=16;
  vizzy_head_publisher.publish(command);
}



void GesturesPanel::updateTopic()
{
  setTopic( output_topic_editor_->text() );
}


void GesturesPanel::updateHeadTopic()
{
  setHeadTopic( output_head_topic_editor_->text() );
}

void GesturesPanel::setHeadTopic(const QString &new_topic)
{
  if (new_topic != output_head_topic_)
  {
    output_head_topic_ = new_topic;
    if(output_head_topic_ == "")
    {
      vizzy_head_publisher.shutdown();      
    }else{
      std::string head_topic = output_head_topic_.toStdString();
      vizzy_head_publisher = nh_.advertise<std_msgs::Int16>(head_topic, 1 );
    }
    
    Q_EMIT configChanged();
  }
}

void GesturesPanel::setTopic(const QString &new_topic)
{
  bothArmsFound = true;
  // Only take action if the name has changed.
  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;
    // If the topic is the empty string, don't publish anything.
    if( output_topic_ == "" )
    {
      vizzy_arm_publisher1.shutdown();
      vizzy_arm_publisher2.shutdown();
    }
    else
    {
      std::string arm1_topic = output_topic_.toStdString();
      std::string arm2_topic = arm1_topic;

      size_t index = 0;
      index = arm2_topic.find("right", 0);
      if (index == std::string::npos)
      {
        index = arm2_topic.find("left", 0);
        if (index == std::string::npos)
        {
          ROS_INFO("Check out Vizzy Arm Gestures topic. I could not find the keyword left or right... So I don't know which arm you are using!");
          bothArmsFound = false; 
        }else{
          arm2_topic.replace(index, 4, "right");
        }
      }else{
        arm2_topic.replace(index, 5, "left");
      }
      
      vizzy_arm_publisher1 = nh_.advertise<std_msgs::Int16>(arm1_topic, 1 );

      if(bothArmsFound)
        vizzy_arm_publisher2 = nh_.advertise<std_msgs::Int16>(arm2_topic, 1 );
    }

    Q_EMIT configChanged();
  }
}

void GesturesPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
  config.mapSetValue( "TopicHead", output_head_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void GesturesPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  QString topichead;
  if( config.mapGetString( "Topic", &topic ))
  {
    output_topic_editor_->setText( topic );
    updateTopic();
  }
  if( config.mapGetString( "TopicHead", &topichead ))
  {
    output_head_topic_editor_->setText( topichead );
    updateHeadTopic();
  }
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vizzy_rviz_plugins::GesturesPanel, rviz::Panel)
