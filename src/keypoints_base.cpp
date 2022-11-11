#include <laser_keypoints/keypoints_base.h>


namespace keypoints_base
{
  bool SetPoint::test(Point target, double w)
  {
  	double delx = point_.x_ - target.x_;
  	double dely = point_.y_ - target.y_;
  	double dist = hypot(delx, dely);
  	if(dist < point_.ri_)
  	{
  	  c_.push_back(target);
  	  if(fabs(delx) < w && fabs(dely) > w)
  	  	cx_.push_back(target);
	    if(fabs(dely) < w && fabs(delx) > w)
  	  	cy_.push_back(target);
  	}
  	return false;
  }

  void SetPoint::formScore()
  {
    double eps = 1;
    score_ = (cx_.size() + cy_.size())/(eps + abs(int(cx_.size()) - int(cy_.size()) ));
  }

  void OCD::visualizePoints()
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;

    cloud.header.frame_id = "r300221673/base_front_laser_link";
    pcl::PointXYZI pcl_point;
    double min = 9999;
    double max = -1;
    for(SetPoint keypoint : keypoints_)
    {
      pcl_point.x = keypoint.point_.x_;//(keypoint.point_.x_, keypoint.point_.y_, 0.0, keypoint.score_);
      pcl_point.y = keypoint.point_.y_;
      pcl_point.z = 0.0;
      if(keypoint.score_ < 1.1)
        continue;
      pcl_point.intensity = keypoint.score_> 250 ? 250:keypoint.score_;
      if(keypoint.score_ < min)
        min = keypoint.score_;
      if(keypoint.score_ > max)
        max = keypoint.score_;
      //ROS_INFO("Point %.2f, %.2f, score %.2f, ri %.2f", keypoint.point_.x_, keypoint.point_.y_, keypoint.score_, keypoint.point_.ri_);
      cloud.points.push_back(pcl_point);
    }
    ROS_INFO("Min score %.2f max score %.2f", min, max);
    cloud_vis_.publish(cloud);
  }

  OCD::OCD()
  {
  	ROS_INFO("Created Orthogonal Corner Detector");
  	par_a_ = 0.2; // Taken from paper
 	  par_b_ = 0.07; // Taken from paper
    par_w_ = 0.05; // Initial guess 
    ros::NodeHandle nh_;
    cloud_vis_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("keypoint_debug", 10);
  }

  void OCD::formSets(sensor_msgs::LaserScan scan)
  {
    double min_ang = scan.angle_min;
    double max_ang = scan.angle_max;
    double inc =  scan.angle_increment;
    double x,y;
    double count = 0;
    keypoints_.clear();
    int skip = 3;
    for(int i = 0; i < scan.ranges.size(); i+=skip) // Use kdtree here if possible
    {
      if(std::isnan(scan.ranges[i]) || scan.ranges[i] > 10)
      	continue;
	    Point point(i, scan.ranges[i]*cos(min_ang+i*inc), scan.ranges[i]*sin(min_ang+i*inc), scan.ranges[i], par_a_, par_b_);
      SetPoint set_p(point);

      for(int j = 0; j < scan.ranges.size(); j++) // Use kdtree here if possible
      {
        count++;
        double complete = skip*count / double(scan.ranges.size() * scan.ranges.size());
        //ROS_INFO("Complete %.2f", complete);
      	if(std::isnan(scan.ranges[j]))
  		  continue;
      	Point target(j, scan.ranges[j]*cos(min_ang+j*inc), scan.ranges[j]*sin(min_ang+j*inc), scan.ranges[j], par_a_, par_b_);
        set_p.test(target, par_w_);
      }
      set_p.formScore();
      keypoints_.push_back(set_p);
    }
  }

};