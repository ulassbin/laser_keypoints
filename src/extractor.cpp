#include <laser_keypoints/extractor.h>

namespace extractor
{
  Extractor::Extractor()
  {
  	ros::NodeHandle nh;
  	ROS_INFO("Initializing extractor");
  	hough::HoughMap temp(-3, 3, 0.1, 0, 2*M_PI, M_PI/100.0);
  	hough_map_ = temp;
    hough_map_.pub_ = nh.advertise<nav_msgs::OccupancyGrid>("hough_map", 5, true);
    laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>("scan",10, &Extractor::messageCallback, this);

    laser_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan_rotated", 10, false);

  }

  void Extractor::messageCallback(const sensor_msgs::LaserScan msg)
  {
    double min_ang = msg.angle_min;
    double max_ang = msg.angle_max;
    double inc =  msg.angle_increment;
    double x,y;
    hough_map_.clear();
    for(int i = 0; i < msg.ranges.size(); i++)
    {
      if(!std::isnan(msg.ranges[i]))
        hough_map_.append(msg.ranges[i]*cos(min_ang+i*inc),msg.ranges[i]*sin(min_ang+i*inc));
    }
    hough_map_.calcStatistics();
    hough_map_.visualize();
    sensor_msgs::LaserScan scan_rotated = msg;
    scan_rotated.angle_min = scan_rotated.angle_min - hough_map_.dominant_ang_;
    scan_rotated.angle_max = scan_rotated.angle_max - hough_map_.dominant_ang_;
    laser_pub_.publish(scan_rotated);
    ocd_detector_.formSets(scan_rotated);
    ocd_detector_.visualizePoints();
  };
}