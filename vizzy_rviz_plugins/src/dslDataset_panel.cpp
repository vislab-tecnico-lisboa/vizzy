/*
Pedro Vicente
based on João Avelino's code
December, 2018
*/

#include <stdio.h>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <math.h> 
#include "../include/vizzy_rviz_plugins/dslDataset_panel.hpp"


namespace vizzy_rviz_plugins {

dslDatasetPanel::dslDatasetPanel(QWidget *parent)
  : rviz::Panel(parent), tfBuffer(), tfListener(tfBuffer)
{

  //Configure the push buttons for the Dataset Recording
  go_to_goal = new QPushButton("Initial Pose!", this);
  home_button = new QPushButton("Home position", this);
  action_button = new QPushButton("Perform action", this);
  recording_button = new QPushButton("Start Recording", this);
  dump_button = new QPushButton("Dump current configuration", this);

  do_all_button = new QPushButton("Do it ALL! ", this);

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


  arm_combobox_ = new QComboBox;
  arm_combobox_->addItem("Left", 0);
  arm_combobox_->addItem("Right", 1);
  arm_label_ = new QLabel(tr("Arm: "));

  //Initialize goal action to update the goal from other nodes (example: ball tracker)
  goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(input_topic_editor_->text().toStdString(), 1, &dslDatasetPanel::poseCallback, this);

  QVBoxLayout* gestures_layout= new QVBoxLayout();
  gestures_layout->addWidget(go_to_goal);
  gestures_layout->addWidget(home_button);
  gestures_layout->addWidget(action_button);
  gestures_layout->addWidget(recording_button);
  gestures_layout->addWidget(dump_button);
  gestures_layout->addWidget(do_all_button);

  QVBoxLayout* controls_out_layout= new QVBoxLayout();

  QGridLayout *controls_layout = new QGridLayout;
  controlsGroup_ = new QGroupBox("Recording/execution parameters"); 

  task_vel_label_ = new QLabel(tr("Velocity (task velocity in y):")); //
  object_label_ = new QLabel(tr("Object ID:"));  
  location_label_ = new QLabel(tr("Location ID:")); //

  repetition_label_ = new QLabel(tr("Repetition Number:")); //
  time_label_ = new QLabel(tr("Execution Time (for action):"));
  trial_label_ = new QLabel(tr("Trial Number:"));


  repetition_spin_ = new QSpinBox();
  repetition_spin_->setRange(1, 20);

  task_vel_spin_ = new QDoubleSpinBox();
  task_vel_spin_->setRange(-0.10, 0.10);
  task_vel_spin_->setSingleStep(0.01);

  location_spin_ = new QSpinBox();
  location_spin_->setRange(1, 15);

  trial_spin_ = new QSpinBox();
  trial_spin_->setRange(0, 500);

  time_spin_ = new QDoubleSpinBox();
  time_spin_->setRange(1, 10);
  time_spin_->setSingleStep(0.1);

  object_spin_ = new QSpinBox();
  object_spin_->setRange(0, 25);

  controls_layout->addWidget(task_vel_label_, 0, 0);
  controls_layout->addWidget(task_vel_spin_, 0, 1);
  controls_layout->addWidget(location_label_, 1, 0);
  controls_layout->addWidget(location_spin_, 1, 1);
  controls_layout->addWidget(repetition_label_, 2, 0);
  controls_layout->addWidget(repetition_spin_, 2, 1);
  controls_layout->addWidget(time_label_, 3, 0);
  controls_layout->addWidget(time_spin_, 3, 1);
  controls_layout->addWidget(trial_label_, 4, 0);
  controls_layout->addWidget(trial_spin_, 4, 1);
  controls_layout->addWidget(object_label_, 5, 0);
  controls_layout->addWidget(object_spin_, 5, 1);

  controls_layout->addWidget(arm_label_, 6, 0);
  controls_layout->addWidget(arm_combobox_,6 , 1);
  controls_layout->addWidget( new QLabel( "Object Name:" ));
  object_name_editor_ = new QLineEdit;
  object_name_editor_->setText("To Be Defined");
  controls_layout->addWidget(object_name_editor_ );

  controls_out_layout->addLayout(controls_layout);
  controlsGroup_->setLayout(controls_out_layout);



  QVBoxLayout* panel_layout = new QVBoxLayout();
  panel_layout->addWidget(controlsGroup_);
  panel_layout->addLayout(gestures_layout);
  panel_layout->addLayout(action_layout);
  panel_layout->addLayout(topic_layout);

  setLayout( panel_layout );

  //Connect objects with signals
  connect( go_to_goal, SIGNAL (released()), this, SLOT(gotoGoal()));
  connect( home_button, SIGNAL (released()), this, SLOT(home()));
  connect( action_button, SIGNAL (released()), this, SLOT (action()));
  connect( recording_button, SIGNAL (released()), this, SLOT (recording()));
  connect( dump_button, SIGNAL (released()), this, SLOT (dumpParameters()));
  connect( do_all_button, SIGNAL (released()), this, SLOT (doAll()));
  

  connect( output_action_editor_, SIGNAL( editingFinished() ), this, SLOT( updateAction() ));
  connect( input_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));

  connect( repetition_spin_, SIGNAL( valueChanged(int) ), this, SLOT( updateRepetition() ));  
  connect( task_vel_spin_, SIGNAL( valueChanged(double) ), this, SLOT( updateTaskVel() ));
  connect( location_spin_, SIGNAL( valueChanged(int) ), this, SLOT( updateLocation() ));
  connect( time_spin_, SIGNAL( valueChanged(double) ), this, SLOT( updateTime() ));
  connect( trial_spin_, SIGNAL( valueChanged(int) ), this, SLOT( updateTrial() ));
  connect( object_spin_, SIGNAL( valueChanged(int) ), this, SLOT( updateObject() ));
  connect( object_name_editor_, SIGNAL( editingFinished() ), this, SLOT( updateObjectName() ));

  connect(arm_combobox_, SIGNAL(currentIndexChanged(int)), this, SLOT(updateArm()));


  updateAction();
  updateTopic();
  initializeParameters();
  updateArm();

  infoPub = nh_.advertise<vizzy_msgs::DslDataset>("datasetInfo", 100);
  csvfile.open("dsl-dataset.csv", std::ios_base::trunc);
  csvfile << "Trial_ID ; Object_ID ; Location_ID ; repetition_Number ; task_vel_y ; movement_duration ; Bag_name" << std::endl;
  csvfile.close();
}

void dslDatasetPanel::initializeParameters()
{

  if(selectedArm == 0) // left
  {
    goal_pos_x_ = -0.30;
    goal_pos_y_ = -0.20;
    goal_pos_z_ = 0.55;
    goal_orient_x_ = 0.04;
    goal_orient_y_ = -0.70;
    goal_orient_z_ = 0.71;
  }else{ // right
    goal_pos_x_ = -0.30;
    goal_pos_y_ = 0.20;
    goal_pos_z_ = 0.55;
    goal_orient_x_ = 0.70;
    goal_orient_y_ = -0.05;
    goal_orient_z_ = 0.02;
  }
  ROS_WARN_STREAM("Initialize Parameters");
  duration_traj_ = 2.5;
  time_spin_->setValue(duration_traj_);
  repetition_number_ = 1;
  repetition_spin_->setValue(repetition_number_);
  location_ = 1;
  location_spin_->setValue(location_);
  linearVelocity_x_ = 0.0;
  linearVelocity_y_ = 0.01;
  task_vel_spin_->setValue(linearVelocity_y_);
  linearVelocity_z_ = 0.0;
  trial_ = 0;
  trial_spin_->setValue(trial_);
  object_ = 0;
  object_spin_->setValue(object_);
  object_names_ = {"bball","lemon","ocup","orange","pear","plego","sycup","wball","yball","ycup","ylego"};
  current_object_name_ = object_names_[object_];
  object_name_editor_->setText(current_object_name_.c_str());

}

void dslDatasetPanel::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_WARN_STREAM("poseCallback");
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PoseStamped onBase;

  try{
  transformStamped = tfBuffer.lookupTransform("base_link", msg->header.frame_id,
                              ros::Time(0));
  tf2::doTransform(*msg, onBase, transformStamped);

  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    return;
  }


  goal_pos_x_ = onBase.pose.position.x;
  goal_pos_y_ = onBase.pose.position.y;
  goal_pos_z_ = onBase.pose.position.z;

  goal_orient_x_ = onBase.pose.orientation.x;
  goal_orient_y_ = onBase.pose.orientation.y;
  goal_orient_z_ = onBase.pose.orientation.z;

  double o_x = goal_orient_x_+goal_orient_x_offset_;
  double o_y = goal_orient_y_+goal_orient_y_offset_;
  double o_z = goal_orient_z_+goal_orient_z_offset_;

  double w = std::sqrt(1.0-(o_x*o_x+o_y*o_y+o_z*o_z));
}

void dslDatasetPanel::gotoGoal()
{
  ROS_WARN_STREAM("Going to Goal");
  vizzy_msgs::CartesianGoal goal;
  goal.type = goal.CARTESIAN;
  goal.end_effector_pose.pose.position.x = goal_pos_x_;
  goal.end_effector_pose.pose.position.y = goal_pos_y_;
  goal.end_effector_pose.pose.position.z = goal_pos_z_;
  

  double o_x = goal_orient_x_;
  double o_y = goal_orient_y_;
  double o_z = goal_orient_z_;
  goal.end_effector_pose.pose.orientation.x = o_x;
  goal.end_effector_pose.pose.orientation.y = o_y;
  goal.end_effector_pose.pose.orientation.z = o_z; 
  goal.end_effector_pose.pose.orientation.w = std::sqrt(1.0-(o_x*o_x+o_y*o_y+o_z*o_z));

  goal.end_effector_pose.header.frame_id = "base_link";//"r_camera_vision_link";  //THIS IS NEEDED?!?
  ROS_WARN_STREAM(goal.end_effector_pose.pose);
  ac->sendGoal(goal);

  bool finished_before_timeout = ac->waitForResult(ros::Duration(10.0)); // waiting 10s

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac->getState();
    ROS_WARN_STREAM("Action finished:" << state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");
}

void dslDatasetPanel::home()
{
  ROS_WARN_STREAM("Going Home");
  vizzy_msgs::CartesianGoal goal;
  goal.type = goal.HOME;
  ac->sendGoal(goal);
  bool finished_before_timeout = ac->waitForResult(ros::Duration(10.0)); // waiting 10s

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac->getState();
    ROS_WARN_STREAM("Action finished: " << state.toString().c_str());
  }
  else
    ROS_WARN_STREAM("Action did not finish before the time out.");
}

void dslDatasetPanel::action()
{
  ROS_WARN_STREAM("Starting Action");
  vizzy_msgs::CartesianGoal goal;
  goal.type = goal.VELOCITY;

  std::vector<float> vel_vec;


  std_msgs::Float32 aux;
  aux.data = linearVelocity_x_;
  goal.velocity.push_back(aux);
  aux.data = linearVelocity_y_;
  goal.velocity.push_back(aux);
  aux.data = linearVelocity_z_;
  goal.velocity.push_back(aux);
  goal.duration.data = duration_traj_;
  goal.end_effector_pose.header.frame_id = "base_link";//"r_camera_vision_link";  //THIS IS NEEDED?!?
  ac->sendGoal(goal);

  bool finished_before_timeout = ac->waitForResult(ros::Duration(10.0)); // waiting 10s

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac->getState();
    ROS_WARN_STREAM("Action finished: " << state.toString().c_str());
  }
  else
    ROS_WARN_STREAM("Action did not finish before the time out.");
}

void dslDatasetPanel::recording()
{
  csvfile.open("dsl-dataset.csv", std::ios_base::app);
  // "Trial_ID ; Object_ID ; Location_ID ; repetition_Number ; task_vel_y ; movement_duration ; Bag_name"
  csvfile << trial_ << ";" << object_ << ";" << location_ << ";" << repetition_number_ << ";" << linearVelocity_y_ << ";" << duration_traj_ << ";dsl-dataset-trial_" << trial_ << ".bag" << std::endl;
  ROS_WARN_STREAM("System Call to record RosBag");
  std::string command;
  command = "rosbag record -O bags/dsl-dataset-trial_" + std::to_string(trial_);// + " --duration=10 /rosout &";
  command += " --duration=20 /datasetInfo /vizzy/joint_states /tf /tf_static /vizzy/l_camera/camera_info /vizzy/l_camera/image_raw/compressed vizzy/r_camera/camera_info /vizzy/r_camera/image_raw/compressed /realsense/color/image_rect_color/compressed /realsense/depth_registered/points /vizzy/left_arm_cartesian_controll/cartesian_action/feedback &";

  vizzy_msgs::DslDataset msg;
  msg.trial_id = trial_;
  msg.object_id = object_;
  msg.object_name = current_object_name_;
  msg.location_id = location_;
  msg.repetition_num = repetition_number_;
  msg.velocity_y = linearVelocity_y_;
  msg.movement_duration = duration_traj_;

  int i = system (command.c_str());
  ros::Duration(2.0).sleep();
  infoPub.publish(msg);
  trial_++;
  trial_spin_->setValue(trial_);
  csvfile.close();
}

void dslDatasetPanel::updateAction()
{
  setAction( output_action_editor_->text() );
}

void dslDatasetPanel::updateObjectName()
{
  current_object_name_ = object_name_editor_->text().toStdString();

  if (object_<object_names_.size()) // Update Object Name
  {
    object_names_[object_] = current_object_name_;
  }
  else // Probably will never enter here
  {
    object_names_.push_back(current_object_name_);
  }
  //

}
void dslDatasetPanel::setAction(const QString &new_action)
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

void dslDatasetPanel::updateTopic()
{
  setTopic( input_topic_editor_->text() );
}

void dslDatasetPanel::setTopic(const QString &new_topic)
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
      goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(input_topic_.toStdString(), 1, &dslDatasetPanel::poseCallback, this);
    }

    Q_EMIT configChanged();
  }
}

void dslDatasetPanel::updateTaskVel()
{
  ROS_WARN_STREAM("updating vel");
  linearVelocity_y_ = (double) (task_vel_spin_->value());
  repetition_number_ = 1;
  repetition_spin_->setValue(repetition_number_);

}

void dslDatasetPanel::updateLocation()
{
  ROS_WARN_STREAM("updating location");
  location_ = (int) (location_spin_->value());
  repetition_number_ = 1;
  repetition_spin_->setValue(repetition_number_);
}

void dslDatasetPanel::updateRepetition()
{
  ROS_WARN_STREAM("updating repetition number");
  repetition_number_ = (int) (repetition_spin_->value());
  
}

void dslDatasetPanel::updateTime()
{
  ROS_WARN_STREAM("updating time");
  duration_traj_ = (double) (time_spin_->value());
  repetition_number_ = 1;
  repetition_spin_->setValue(repetition_number_);
  
}
void dslDatasetPanel::updateTrial()
{
  ROS_WARN_STREAM("updating trial Number");
  trial_ = (double) (trial_spin_->value());
  
}

void dslDatasetPanel::updateObject()
{
  ROS_WARN_STREAM("updating Object");
  object_ = (double) (object_spin_->value());
  repetition_number_ = 1;
  repetition_spin_->setValue(repetition_number_);
  if (object_<object_names_.size()) // Known Object! HARD-CODED
  {
    object_name_editor_->setText(object_names_[object_].c_str());
  }
  else
  {
    object_name_editor_->setText("unk obj");
    object_names_.push_back("unkObj_" + std::to_string(object_));
  }
  current_object_name_ = object_names_[object_];
  
}

void dslDatasetPanel::dumpParameters()
{
  ROS_WARN_STREAM("Dumping Current Parameters");
  ROS_WARN_STREAM("Time: " << duration_traj_);
  ROS_WARN_STREAM("Repetition Number: " << repetition_number_);
  ROS_WARN_STREAM("Location id: " << location_);
  ROS_WARN_STREAM("Object id: " << object_);
  ROS_WARN_STREAM("Object name: " << current_object_name_);
  ROS_WARN_STREAM("Linear Velocity y: " << linearVelocity_y_);
  ROS_WARN_STREAM("Linear Velocity x: " << linearVelocity_x_);
  ROS_WARN_STREAM("Linear Velocity z: " << linearVelocity_z_);
  ROS_WARN_STREAM("Trial Number: " << trial_);
  
}
void dslDatasetPanel::doAll()
{
  ROS_WARN_STREAM("Doing everything!");
  recording();
  gotoGoal();
  action();
  home();
  repetition_number_++;
  repetition_spin_->setValue(repetition_number_);
  ROS_WARN_STREAM("Done!");


}
void dslDatasetPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Action", output_action_ );
  config.mapSetValue( "Topic", input_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void dslDatasetPanel::load( const rviz::Config& config )
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

void dslDatasetPanel::updateArm()
{
  selectedArm = arm_combobox_->currentIndex();

  if(selectedArm == 0)
  {
    output_action_editor_->setText("/vizzy/left_arm_cartesian_controller/cartesian_action");
    updateAction();
  }else
  {
    output_action_editor_->setText("/vizzy/right_arm_cartesian_controller/cartesian_action");
    updateAction();
  }
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vizzy_rviz_plugins::dslDatasetPanel, rviz::Panel)
