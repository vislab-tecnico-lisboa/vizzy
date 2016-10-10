/*
 * LICENSE: https://github.com/utexas-bwi/segbot/blob/devel/LICENSE
 */

#ifndef LASER_SCAN_ANGLE_FILTER_H
#define LASER_SCAN_ANGLE_FILTER_H

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>


namespace vizzy_sensors
{
  class ScanAngleFilter : public filters::FilterBase<sensor_msgs::LaserScan>
  {
    public:

      bool configure() {
        return true;
      }

      virtual ~ScanAngleFilter(){}

      bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan){

        filtered_scan.ranges.resize(input_scan.ranges.size());

	double angle_step=(filtered_scan.angle_max-filtered_scan.angle_min) / input_scan.ranges.size();
	double min_angle=-0.4;
	double max_angle= 0.4;
	getParam("min_angle", min_angle);
	getParam("max_angle", max_angle);

	//getParam("footprint_frame", footprint_frame_);
        for(unsigned int count = 0; count < input_scan.ranges.size(); ++count){
	  double angle_=angle_step* (double)count+ (double)input_scan.angle_min;

	  if(angle_<min_angle || angle_>max_angle)
	  {
		filtered_scan.ranges[count]=0.0;
 	  }
	  else
	  {
		filtered_scan.ranges[count]=input_scan.ranges[count];
	  }

        }

        //make sure to set all the needed fields on the filtered scan
        filtered_scan.header.frame_id = input_scan.header.frame_id;
        filtered_scan.header.stamp = input_scan.header.stamp;
        filtered_scan.angle_min = input_scan.angle_min;
        filtered_scan.angle_max = input_scan.angle_max;
        filtered_scan.angle_increment = input_scan.angle_increment;
        filtered_scan.time_increment = input_scan.time_increment;
        filtered_scan.scan_time = input_scan.scan_time;
        filtered_scan.range_min = input_scan.range_min;
        filtered_scan.range_max = input_scan.range_max;
        filtered_scan.intensities = input_scan.intensities;

        return true;

      }
  };
}
#endif

