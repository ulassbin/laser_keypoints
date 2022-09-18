#include <laser_keypoints/hough.h>

#include <sensor_msgs/LaserScan.h>

namespace extractor
{
  class Extractor
  {
    public:
    Extractor();
    private:
    void messageCallback(const sensor_msgs::LaserScan msg);

    hough::HoughMap hough_map_;
    ros::Subscriber laser_sub_;
  };
}