#include <laser_keypoints/keypoints_base.h>


namespace keypoints_base
{
  bool SetPoint::test(Point target)
  {
  	double delx = point_.x_ - target.x_;
  	double dely = point.y_ - target.y_;
  	double dist = hypot(delx, dely);
  	if(dist < point.ri_)
  	{
  	  c_.push_back(target);
  	  if(fabs(delx) < par_w_ && fabs(dely) > w)
  	  	cx_.push_back(target);
	  if(fabs(dely) < par_w_ && fabs(delx) > w)
  	  	cy_.push_back(target);

  	}
  	return false;
  }

  OCD::OCD()
  {
  	ROS_INFO("Created Orthogonal Corner Detector");
  	double par_a_ = 0.2; // Taken from paper
 	double par_b_ = 0.07; // Taken from paper
    double par_w_ = 0.2; // Initial guess 
  }

  void OCD::formSets(sensor_msgs::LaserScan scan)
  {
    double min_ang = scan.angle_min;
    double max_ang = scan.angle_max;
    double inc =  scan.angle_increment;
    double x,y;
    hough_map_.clear();
    keypoints_.clear();
    for(int i = 0; i < scan.ranges.size(); i++) // Use kdtree here if possible
    {
      if(std::isnan(scan.ranges[i]))
      	continue;
	  Point point(scan.ranges[i]*cos(min_ang+i*inc),scan.ranges[i]*sin(min_ang+i*inc), par_a_, par_b_);
      for(int j = 0; j < scan.ranges.size(); j++) // Use kdtree here if possible
      {
      	if(std::isnan(scan.ranges[j]))
  		  continue;
      	Point compared(scan.ranges[j]*cos(min_ang+j*inc),scan.ranges[j]*sin(min_ang+j*inc));


      }
    }
  }

};