#include <laser_keypoints/hough.h>

namespace hough
{
  HoughMap::HoughMap()
  {
    ROS_INFO("Constructed blank HoughMap");
  }

  HoughMap::HoughMap(double min_r, double max_r, double r_step,
  	double min_th, double max_th, double th_step)
  {
  	// origin is 0,0 assumed here.
  	width_ = (max_r-min_r)/r_step + 1;
  	height_ = (max_th - min_th)/th_step + 1;
  	r_res_ = r_step;
  	th_res_ = th_step;
  	size_ = width_ * height_;

    min_r_ = min_r;
    min_th_ = min_th;

  	map_.resize(size_);
  	int r_index;
  	int th_index;
  	for(int i = 0; i<map_.size(); i++)
  	{
  	  r_index = i % width_;
  	  th_index = i / width_;
  	  map_[i].r_ = r_index * r_step;
  	  map_[i].th_ = th_index * th_step;
  	  map_[i].count_ = 0;
  	}
  }


  void HoughMap::append(double x, double y)
  {
    for(int i = 0; i < map_.size(); i++)
    {
      if(map_[i].getDistance(x, y) < dist_thresh_)
        map_[i].count_++;
    }
  }

  void HoughMap::calcStatistics()
  {
    int th_index(0), max_index(0), max_count(0);
    int dom_ang(0), max_ang_count(0);
    std::vector<int> ang_sums_;
    ang_sums_.resize(height_);
    for(int i = 0; i < ang_sums_.size(); i++)
      ang_sums_[i] = 0;
    for(int i = 0; i < map_.size(); i++)
    {
      if(map_[i].count_ > max_count)
      {
        max_count = map_[i].count_;
        max_index = i;
      }
      th_index = i / width_;
      ang_sums_[th_index] += map_[i].count_;
    }

    for(int i = 0; i<ang_sums_.size(); i++)
    {
      if(ang_sums_[i] > max_ang_count)
      {
        max_ang_count = ang_sums_[i];
        dom_ang = i;
      }
    }

    dominant_ang_ = min_th_ + th_res_*dom_ang;
    max_count_ = max_count;
    ROS_INFO("Dominant angle is %.2f with count %d", dominant_ang_, max_ang_count);

    double r = min_r_ + max_index % width_ * r_res_;
    double th = min_th_ + max_index / width_ * th_res_;
    ROS_INFO("Dominant position is %.2f, %.2f with count %d", r, th, max_count);
  }

  void HoughMap::clear()
  {
    for(int i = 0; i < map_.size(); i++)
    {
    	map_[i].count_ = 0;
    }
  }

  void HoughMap::visualize()
  {
  	nav_msgs::OccupancyGrid vis_msg_;
  	vis_msg_.header.stamp = ros::Time::now();
  	vis_msg_.header.frame_id = "map";
  	vis_msg_.info.resolution = 0.05; // Not important
  	vis_msg_.info.width = width_;
  	vis_msg_.info.height = height_;
  	vis_msg_.info.origin.position.x = 0.0;
  	vis_msg_.info.origin.position.y = 0.0;
  	vis_msg_.info.origin.position.z = 0.0;
  	vis_msg_.info.origin.orientation.x = 0.0;
  	vis_msg_.info.origin.orientation.y = 0.0;
  	vis_msg_.info.origin.orientation.z = 0.0;
  	vis_msg_.info.origin.orientation.w = 1.0;
  	vis_msg_.data.resize(width_*height_);

    for(int i = 0; i < map_.size(); i++)
    {
      vis_msg_.data[i] = 255*float(map_[i].count_)/float(max_count_);
    }
  	pub_.publish(vis_msg_);
  }
}