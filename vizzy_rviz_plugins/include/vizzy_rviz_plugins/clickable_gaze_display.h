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

#ifndef CLICKABLE_GAZE_DISPLAY_H
#define CLICKABLE_GAZE_DISPLAY_H

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <OgreMaterial.h>
#include <OgreRenderTargetListener.h>
#include <OgreSharedPtr.h>

#include "rviz/image/image_display_base.h"
#include "rviz/image/ros_image_texture.h"
#include "rviz/render_panel.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <vizzy_msgs/GazeAction.h>

#include "rviz/properties/bool_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#endif

namespace Ogre {
class SceneNode;
class Rectangle2D;
}

namespace vizzy_rviz_plugins
{
typedef boost::function<void(QMouseEvent *)> MouseEventHandler;

class InteractiveRenderPanel : public rviz::RenderPanel {
  Q_OBJECT
public:
  InteractiveRenderPanel(QWidget *parent = 0) : RenderPanel(parent) {}

  virtual void mouseMoveEvent(QMouseEvent *event);
  virtual void mousePressEvent(QMouseEvent *event);
  virtual void mouseReleaseEvent(QMouseEvent *event);
  virtual void mouseDoubleClickEvent(QMouseEvent *event);

Q_SIGNALS:
  void mouseEventHandler(QMouseEvent *event);

private:
  MouseEventHandler mouse_event_handler_;
};
/**
 * \class ClickableGazeDisplay
 *
 */
class ClickableGazeDisplay : public rviz::ImageDisplayBase {
  Q_OBJECT
public:
  ClickableGazeDisplay();
  virtual ~ClickableGazeDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

public Q_SLOTS:
  virtual void updateNormalizeOptions();
  void mouseEventHandler(QMouseEvent *event);

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  /* This is called by incomingMessage(). */
  virtual void processMessage(const sensor_msgs::Image::ConstPtr &msg);

private:
  void clear();
  void updateStatus();
  void updateTopic();

  Ogre::SceneManager *img_scene_manager_;
  Ogre::SceneNode *img_scene_node_;
  Ogre::Rectangle2D *screen_rect_;
  Ogre::MaterialPtr material_;

  rviz::ROSImageTexture texture_;

  InteractiveRenderPanel *render_panel_;

  BoolProperty *normalize_property_;
  rviz::FloatProperty *min_property_;
  rviz::FloatProperty *max_property_;
  rviz::IntProperty *median_buffer_size_property_;
  bool got_float_image_;

  rviz::StringProperty *topic_property_;

  ros::NodeHandle nh_;
  ros::Publisher pub_;

  float img_scale_;
  float img_width_;
  float img_height_;
  float img_xoffset_;
  float img_yoffset_;
  
  std::string camera_info_topic;

  std::vector<float> K_ = {0, 0, 0, 0, 0, 0, 0, 0};

  actionlib::SimpleActionClient<vizzy_msgs::GazeAction> ac;
};

}
#endif