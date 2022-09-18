#include <ros/ros.h>

#include <vector>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace hough
{
  struct HParam
  {
    double r_;
    double th_;
    int count_;
    double getDistance(double x, double y)
    {
      // r_ = x*cos(th_)+y*sin(th_)
      return fabs(x*cos(th_)+y*sin(th_)-r_);
    }
  };

  struct HoughMap
  {
  	int width_;
  	int height_;
  	double r_res_;
  	double th_res_;
  	int size_;
  	double dist_thresh_ = 0.01;
    double min_r_;
    double min_th_;

  	std::vector<HParam> map_;
  	ros::Publisher pub_;

    int max_count_;
    double dominant_ang_;


  	HoughMap(double min_r, double max_r, double r_step,
  	double min_th, double max_th, double th_step);

    HoughMap();

	  void calcStatistics();
    void append(double x, double y);
  	void clear();
  	void visualize();
  };
};