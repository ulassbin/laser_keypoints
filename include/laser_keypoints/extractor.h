#include <laser_keypoints/hough.h>
#include <laser_keypoints/keypoints_base.h>

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
    ros::Publisher laser_pub_;
    keypoints_base::OCD ocd_detector_;
  };
}