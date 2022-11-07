#include <sensor_msgs/LaserScan.h>

namespace keypoints_base
{
  struct SetPoint
  {
    Point point_; // pi
    std::vector<Point> c_; // < ri 
    std::vector<Point> cx_; //  |pj,x − pi,x| < w ∧ |pj,y − pi,y | > w for items in C
    std::vector<Point> cy_; //  |pj,y − pi,y | < w ∧ |pj,x − pi,x| > w for items in C
    SetPoint(Point p) : point_(p) {};
    bool test(Point p_compare);
  };

  struct Point
  {
    Point(Point p) : id_(p.id_), r_(p.r_), x_(p.x_), y_(p.y_), ri_(p.ri_) {}; 

    Point(int id, double x, double y, double r, double par_a, double par_b) : id_(id), x_(x), y_(y), r_(r)
    {
      ri_ = par_a * exp(par_b * fabs(r));
    };

    Point(int id, double x, double y, double r) : id_(id), x_(x), y_(y), r_(r), ri_(r) {};

    int id_;
    double r_;
    double x_;
    double y_;
    double ri_;
  };

  class OCD // Orthogonal Corner detector
  {
    public:
      OCD();
      void formSets(sensor_msgs::LaserScan scan);
      
      double par_a_;
      double par_b_;
      double par_w_;
    private:
      std::vector<SetPoint> keypoints_;
  }

}