#include "../include/vizzy_rviz_plugins/wasd_teleop_tool.h"

namespace vizzy_rviz_plugins {

WasdTeleopTool::WasdTeleopTool() : linear_velocity_(0), angular_velocity_(0), output_topic_(""),
  boosted_ang_(0), boosted_lin_(0)
{

  shortcut_key_ = '<';

}

WasdTeleopTool::~WasdTeleopTool()
{
  velocity_publisher_.shutdown();
}

void WasdTeleopTool::onInitialize()
{
  setName( "WASD Teleop" );

  topic_property_ = new rviz::RosTopicProperty("cmd_vel topic", "/vizzy/cmd_vel",
                                               QString::fromStdString(ros::message_traits::datatype<geometry_msgs::Twist>()),
                                               "geometry_msgs::Twist topic to publish to.", getPropertyContainer(), SLOT( updateTopic()), this);

  move_base_cancel_topic_property_ = new rviz::RosTopicProperty("Action Cancel Topic", "move_base/cancel",
                                                                QString::fromStdString(ros::message_traits::datatype<geometry_msgs::Twist>()),
                                                                "actionlib_msgs::GoalID topic to publish the cancel command.", getPropertyContainer(), SLOT( updateActionCancelTopic()), this);

  max_lin_property_ = new rviz::FloatProperty( "Max linear velocity", 2.0,
                          "Maximum absolute value of the linear velocity (m/s)",
                          getPropertyContainer(), SLOT( maxLinearVelUpdate() ), this);

  max_ang_property_ = new rviz::FloatProperty( "Max angular velocity", M_PI/4,
                          "Maximum absolute value of the angular velocity (rad/s)",
                          getPropertyContainer(), SLOT( maxAngularVelUpdate() ), this);

  linear_velocity_ = 0;
  angular_velocity_ = 0;
  boosted_ang_ = 0;
  boosted_lin_ = 0;

}




void WasdTeleopTool::activate()
{

  qApp->installEventFilter(this);

  output_timer = new QTimer( this );

  access_all_keys_ = true;

  updateTopic();
  updateActionCancelTopic();

  connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  // Start the timer.
  output_timer->start( 100 );

  actionlib_msgs::GoalID cancelAllGoals;
  goal_cancel_publisher_.publish(cancelAllGoals);


}

void WasdTeleopTool::deactivate()
{
  linear_velocity_ = 0;
  angular_velocity_ = 0;
  boosted_lin_ = 0;
  boosted_ang_ = 0;
  output_timer->stop();
  access_all_keys_ = false;
  qApp->removeEventFilter(this);
}


bool WasdTeleopTool::eventFilter(QObject *obj, QEvent *event) //AWESOME!
{
  if (event->type() == QEvent::KeyPress)
  {
    QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
    if(keyEvent->key() == Qt::Key_W && !keyEvent->isAutoRepeat())
    {
      linear_velocity_+=lin_step_;
      return true;
    }
    else if(keyEvent->key() == Qt::Key_S && !keyEvent->isAutoRepeat())
    {
      linear_velocity_-=lin_step_;
      return true;
    }
    else if(keyEvent->key() == Qt::Key_A && !keyEvent->isAutoRepeat())
    {
      angular_velocity_+=ang_step_;
      return true;
    }
    else if(keyEvent->key() == Qt::Key_D && !keyEvent->isAutoRepeat())
    {
      angular_velocity_-=ang_step_;
      return true;
    }
    else if(keyEvent->key() == Qt::Key_Shift && !keyEvent->isAutoRepeat())
    {
      boosted_lin_ = 1;
      return true;
    }
    else if(keyEvent->key() == Qt::Key_Alt && !keyEvent->isAutoRepeat())
    {
      boosted_ang_ = 1;
      return true;
    }

  }else if (event->type() == QEvent::KeyRelease)
  {

    QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
    if(keyEvent->key() == Qt::Key_W && !keyEvent->isAutoRepeat())
    {

      linear_velocity_-=lin_step_;
      return true;
    }
    else if(keyEvent->key() == Qt::Key_S && !keyEvent->isAutoRepeat())
    {
      linear_velocity_+=lin_step_;
      return true;
    }
    else if(keyEvent->key() == Qt::Key_A && !keyEvent->isAutoRepeat())
    {
      angular_velocity_-=ang_step_;
      return true;
    }
    else if(keyEvent->key() == Qt::Key_D && !keyEvent->isAutoRepeat())
    {
      angular_velocity_+=ang_step_;
      return true;
    }
    else if(keyEvent->key() == Qt::Key_Shift && !keyEvent->isAutoRepeat())
    {
      boosted_lin_ = 0;
      return true;
    }
    else if(keyEvent->key() == Qt::Key_Alt && !keyEvent->isAutoRepeat())
    {
      boosted_ang_ = 0;
      return true;
    }

  }

  return QObject::eventFilter(obj, event);
}

void WasdTeleopTool::maxLinearVelUpdate()
{
  lin_step_ = max_lin_property_->getFloat();
}

void WasdTeleopTool::maxAngularVelUpdate()
{
  ang_step_ = max_ang_property_->getFloat();
}

void WasdTeleopTool::sendVel()
{
  if( ros::ok() && velocity_publisher_ )
  {
    geometry_msgs::Twist msg;

    if(boosted_lin_)
      msg.linear.x = linear_velocity_*2;
    else
      msg.linear.x = linear_velocity_;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    if(boosted_ang_)
      msg.angular.z = angular_velocity_*2;
    else
      msg.angular.z = angular_velocity_;

    velocity_publisher_.publish( msg );
  }

}


void WasdTeleopTool::setVel( float lin, float ang )
{
  linear_velocity_ = lin;
  angular_velocity_ = ang;
}

void WasdTeleopTool::updateTopic()
{
  setTopic(topic_property_->getTopic());
}

void WasdTeleopTool::updateActionCancelTopic()
{
  setActionCancelTopic(move_base_cancel_topic_property_->getTopic());
}

void WasdTeleopTool::setActionCancelTopic(const QString& new_topic)
{
  // Only take action if the name has changed.
  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;

    // If the topic is the empty string, don't publish anything.
    if( output_topic_.toStdString() == "" )
    {
      goal_cancel_publisher_.shutdown();
    }
    else
    {
      goal_cancel_publisher_ = nh_.advertise<actionlib_msgs::GoalID>( output_topic_.toStdString(), 1 );
    }
  }
}

void WasdTeleopTool::setTopic( const QString& new_topic )
{

  // Only take action if the name has changed.
  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;

    // If the topic is the empty string, don't publish anything.
    if( output_topic_.toStdString() == "" )
    {
      velocity_publisher_.shutdown();
    }
    else
    {
      velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( output_topic_.toStdString(), 1 );
    }
  }
}


// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void WasdTeleopTool::save( rviz::Config config ) const
{
  rviz::Tool::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}


// Load all configuration data for this panel from the given Config object.
void WasdTeleopTool::load( const rviz::Config& config )
{
  rviz::Tool::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    output_topic_ = topic;
    updateTopic();
  }
}

} //end namespace vizzy_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vizzy_rviz_plugins::WasdTeleopTool, rviz::Tool)

