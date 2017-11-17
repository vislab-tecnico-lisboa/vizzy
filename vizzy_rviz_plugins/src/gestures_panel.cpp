#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include "../include/vizzy_rviz_plugins/gestures_panel.hpp"


namespace vizzy_rviz_plugins {

GesturesPanel::GesturesPanel(Qwidget *parent)
  : rviz::Panel(parent)
{
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Output Topic:" ));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_ );

  QVBoxLayout* gestures_layout = QVBoxLayout;


}

}
