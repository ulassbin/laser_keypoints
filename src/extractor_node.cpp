#include <laser_keypoints/extractor.h>


int main(int argc, char** argv)
{
  ros::init(argc,argv, "laser_keypoints_node");
  ros::NodeHandle nh;
  extractor::Extractor extract;
  ros::spin();
}