#include <sensor_msgs/LaserScan.h>

#include <pcl_ros/point_cloud.h>

namespace keypoints_base
{

  struct Point
  {

    Point(int id, double x, double y, double r, double par_a, double par_b) : id_(id), x_(x), y_(y), r_(r)
    {
      ri_ = par_a * exp(par_b * r_);
    };

    Point(int id, double x, double y, double r) : id_(id), x_(x), y_(y), r_(r), ri_(r) {};

    Point(const Point& p) : id_(p.id_), r_(p.r_), x_(p.x_), y_(p.y_), ri_(p.ri_) {}; 

    int id_;
    double r_;
    double x_;
    double y_;
    double ri_;
  };

  struct SetPoint
  {
    Point point_; // pi
    double score_;
    std::vector<Point> c_; // < ri 
    std::vector<Point> cx_; //  |pj,x − pi,x| < w ∧ |pj,y − pi,y | > w for items in C
    std::vector<Point> cy_; //  |pj,y − pi,y | < w ∧ |pj,x − pi,x| > w for items in C
    SetPoint(Point p) : point_(p) {};
    bool test(Point p_compare, double w);
    void formScore();
  };

  class OCD // Orthogonal Corner detector
  {
    public:
      OCD();
      void formSets(sensor_msgs::LaserScan scan);
      void visualizePoints();

    private:
      double par_a_;
      double par_b_;
      double par_w_;
      std::vector<SetPoint> keypoints_;
      ros::Publisher cloud_vis_;
  };

}