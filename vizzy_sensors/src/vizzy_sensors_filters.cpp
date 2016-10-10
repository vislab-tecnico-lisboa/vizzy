/*
 * LICENSE: https://github.com/utexas-bwi/segbot/blob/devel/LICENSE
 */

#include "vizzy_sensors/footprint_filter.h"
#include "vizzy_sensors/nan_to_inf_filter.h"
#include "vizzy_sensors/scan_angle_filter.h"
#include "sensor_msgs/LaserScan.h"
#include "filters/filter_base.h"

#include "pluginlib/class_list_macros.h"

PLUGINLIB_REGISTER_CLASS(vizzy_sensors/VizzyFootprintFilter, vizzy_sensors::VizzyFootprintFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_REGISTER_CLASS(vizzy_sensors/NanToInfFilter, vizzy_sensors::NanToInfFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_REGISTER_CLASS(vizzy_sensors/ScanAngleFilter, vizzy_sensors::ScanAngleFilter, filters::FilterBase<sensor_msgs::LaserScan>)
