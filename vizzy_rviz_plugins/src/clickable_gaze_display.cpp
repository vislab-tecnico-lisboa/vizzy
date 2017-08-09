/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <QMouseEvent>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <OgreTechnique.h>
#include <OgreCamera.h>

#include <tf/transform_listener.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/render_panel.h"
#include "rviz/validate_floats.h"

#include <cstdio>

#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/CameraInfo.h"

#include "../include/vizzy_rviz_plugins/clickable_gaze_display.h"

namespace vizzy_rviz_plugins{

ClickableGazeDisplay::ClickableGazeDisplay()
    : ImageDisplayBase(), texture_(), img_scale_(0), img_xoffset_(0),
      img_yoffset_(0), img_width_(0), img_height_(0), ac("gaze", true) {
  normalize_property_ = new BoolProperty(
      "Normalize Range", true, "If set to true, will try to estimate the range "
                               "of possible values from the received images.",
      this, SLOT(updateNormalizeOptions()));

  min_property_ = new rviz::FloatProperty("Min Value", 0.0,
                                    "Value which will be displayed as black.",
                                    this, SLOT(updateNormalizeOptions()));

  max_property_ = new rviz::FloatProperty("Max Value", 1.0,
                                    "Value which will be displayed as white.",
                                    this, SLOT(updateNormalizeOptions()));

  median_buffer_size_property_ = new rviz::IntProperty(
      "Median window", 5,
      "Window size for median filter used for computin min/max.", this,
      SLOT(updateNormalizeOptions()));

  topic_property_ = new rviz::StringProperty("Topic", "/image_clicked_point",
                                       "The topic on which to publish points.",
                                       this, SLOT(updateTopic()));

  updateTopic();

  got_float_image_ = false;
}

void ClickableGazeDisplay::onInitialize() {
  ImageDisplayBase::onInitialize();
  {
    static uint32_t count = 0;
    std::stringstream ss;
    ss << "ClickableGazeDisplay" << count++;
    img_scene_manager_ = Ogre::Root::getSingleton().createSceneManager(
        Ogre::ST_GENERIC, ss.str());
  }

  img_scene_node_ =
      img_scene_manager_->getRootSceneNode()->createChildSceneNode();

  {
    static int count = 0;
    std::stringstream ss;
    ss << "ClickableGazeDisplayObject" << count++;

    screen_rect_ = new Ogre::Rectangle2D(true);
    screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
    screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

    ss << "Material";
    material_ = Ogre::MaterialManager::getSingleton().create(
        ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_->setSceneBlending(Ogre::SBT_REPLACE);
    material_->setDepthWriteEnabled(false);
    material_->setReceiveShadows(false);
    material_->setDepthCheckEnabled(false);

    material_->getTechnique(0)->setLightingEnabled(false);
    Ogre::TextureUnitState *tu =
        material_->getTechnique(0)->getPass(0)->createTextureUnitState();
    tu->setTextureName(texture_.getTexture()->getName());
    tu->setTextureFiltering(Ogre::TFO_NONE);

    material_->setCullingMode(Ogre::CULL_NONE);
    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    screen_rect_->setBoundingBox(aabInf);
    screen_rect_->setMaterial(material_->getName());
    img_scene_node_->attachObject(screen_rect_);
  }

  MouseEventHandler mevent =
      boost::bind(&ClickableGazeDisplay::mouseEventHandler, this, _1);
  render_panel_ = new InteractiveRenderPanel();
  connect(render_panel_, SIGNAL(mouseEventHandler(QMouseEvent *)), this,
          SLOT(mouseEventHandler(QMouseEvent *)));
  render_panel_->getRenderWindow()->setAutoUpdated(false);
  render_panel_->getRenderWindow()->setActive(false);

  render_panel_->resize(640, 480);
  render_panel_->initialize(img_scene_manager_, context_);

  setAssociatedWidget(render_panel_);

  render_panel_->setAutoRender(false);
  render_panel_->setOverlaysEnabled(false);
  render_panel_->getCamera()->setNearClipDistance(0.01f);

  updateNormalizeOptions();
}

ClickableGazeDisplay::~ClickableGazeDisplay() {
  if (initialized()) {
    delete render_panel_;
    delete screen_rect_;
    img_scene_node_->getParentSceneNode()->removeAndDestroyChild(
        img_scene_node_->getName());
  }
}

void ClickableGazeDisplay::onEnable() {
  ImageDisplayBase::subscribe();
  render_panel_->getRenderWindow()->setActive(true);
}

void ClickableGazeDisplay::onDisable() {
  render_panel_->getRenderWindow()->setActive(false);
  ImageDisplayBase::unsubscribe();
  clear();
}

void ClickableGazeDisplay::updateNormalizeOptions() {
  if (got_float_image_) {
    bool normalize = normalize_property_->getBool();

    normalize_property_->setHidden(false);
    min_property_->setHidden(normalize);
    max_property_->setHidden(normalize);
    median_buffer_size_property_->setHidden(!normalize);

    texture_.setNormalizeFloatImage(normalize, min_property_->getFloat(),
                                    max_property_->getFloat());
    texture_.setMedianFrames(median_buffer_size_property_->getInt());
  } else {
    normalize_property_->setHidden(true);
    min_property_->setHidden(true);
    max_property_->setHidden(true);
    median_buffer_size_property_->setHidden(true);
  }
}

void ClickableGazeDisplay::clear() {
  texture_.clear();

  if (render_panel_->getCamera()) {
    render_panel_->getCamera()->setPosition(
        Ogre::Vector3(999999, 999999, 999999));
  }
}

void ClickableGazeDisplay::update(float wall_dt, float ros_dt) {
  try {
    texture_.update();

    // make sure the aspect ratio of the image is preserved
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    float img_width = texture_.getWidth();
    float img_height = texture_.getHeight();

    if (img_width != 0 && img_height != 0 && win_width != 0 &&
        win_height != 0) {
      float img_aspect = img_width / img_height;
      float win_aspect = win_width / win_height;

      if (img_aspect > win_aspect) {
        img_scale_ = img_width / win_width;
        img_xoffset_ = 0;
        img_yoffset_ = (win_height-img_height/img_scale_)/2;

        screen_rect_->setCorners(-1.0f, 1.0f * win_aspect / img_aspect, 1.0f,
                                 -1.0f * win_aspect / img_aspect, false);
      } else {
        img_scale_ = img_height / win_height;
        img_xoffset_ = (win_width-img_width/img_scale_)/2;
        img_yoffset_ = 0;

        screen_rect_->setCorners(-1.0f * img_aspect / win_aspect, 1.0f,
                                 1.0f * img_aspect / win_aspect, -1.0f, false);
      }
      img_width_ = img_width;
      img_height_ = img_height;
    }

    render_panel_->getRenderWindow()->update();
  } catch (rviz::UnsupportedImageEncoding &e) {
    setStatus(rviz::StatusProperty::Error, "Image", e.what());
  }
}

void ClickableGazeDisplay::reset() {
  ImageDisplayBase::reset();
  clear();
}

/* This is called by incomingMessage(). */
void ClickableGazeDisplay::processMessage(const sensor_msgs::Image::ConstPtr &msg) {
  bool got_float_image =
      msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
      msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      msg->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
      msg->encoding == sensor_msgs::image_encodings::MONO16;

  if (got_float_image != got_float_image_) {
    got_float_image_ = got_float_image;
    updateNormalizeOptions();
  }
  texture_.addMessage(msg);
}

void ClickableGazeDisplay::mouseEventHandler(QMouseEvent *mevent) {
  if (mevent->button() == Qt::NoButton)
    return;
  else{

        std::string delimiter = "/image";
        std::string topic_prefix = sub_->getTopic().substr(0, sub_->getTopic().find(delimiter));
        camera_info_topic = topic_prefix + "/camera_info";
        sensor_msgs::CameraInfoConstPtr l_camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic, ros::Duration(30));

        double fx = 1;
        double fy = 1;
        double cx = 0;
        double cy = 0;

        if (l_camera_info == NULL)
          {
            ROS_ERROR_STREAM("Failed to get camera parameters!");
            return;
          }else{
            fx = l_camera_info->K.at(0);
            fy = l_camera_info->K.at(4);
            cx = l_camera_info->K.at(2);
            cy = l_camera_info->K.at(5);
          }

        geometry_msgs::PointStamped ps;

        ps.point.x = (mevent->pos().x() - img_xoffset_) * img_scale_;
        ps.point.y = (mevent->pos().y() - img_yoffset_) * img_scale_;

        if(ps.point.x < 0)
          return;

        if(ps.point.y < 0)
          return;
        
        ps.point.z = 0;

        if (ps.point.x > img_width_)
          ps.point.x = img_width_;
        else if (ps.point.y > img_height_)
          ps.point.y = img_height_;


    if(mevent->button() == Qt::LeftButton)
      {

        //Assuming the target is at 1.5m from the camera

        vizzy_msgs::GazeGoal goal;
        goal.type=vizzy_msgs::GazeGoal::CARTESIAN;
        goal.fixation_point.point.x = 1.5; //(Relatively to the camera link, which has the x axis to the front)
        goal.fixation_point.point.y = -1.5*(ps.point.x-cx)/fx;
        goal.fixation_point.point.z = -1.5*(ps.point.y-cy)/fy;
        goal.fixation_point_error_tolerance = 0.01;

        goal.fixation_point.header.frame_id=l_camera_info->header.frame_id;
        goal.fixation_point.header.stamp=ros::Time::now();

        ac.sendGoal(goal);


      }else if(mevent->button() == Qt::MiddleButton)
      {


      vizzy_msgs::GazeGoal goal;
      goal.type=vizzy_msgs::GazeGoal::HOME;
      goal.fixation_point_error_tolerance = 0.01;

      goal.fixation_point.header.frame_id=l_camera_info->header.frame_id;
      goal.fixation_point.header.stamp=ros::Time::now();

      ac.sendGoal(goal);
        

      }else if(mevent->button() == Qt::RightButton)
      {

        ps.header.frame_id = "camera_vision_link";
        ps.header.stamp = ros::Time::now();
        pub_.publish(ps);
      
      }

  } 
  

}
void ClickableGazeDisplay::updateTopic() {
  pub_ = nh_.advertise<geometry_msgs::PointStamped>(
      topic_property_->getStdString(), 1);
}

void InteractiveRenderPanel::mouseReleaseEvent(QMouseEvent *event) {
  Q_EMIT mouseEventHandler(event);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vizzy_rviz_plugins::ClickableGazeDisplay, rviz::Display)
