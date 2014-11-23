#include <ros.h>
#include <costmap_2d/costmap_2d.h>

int main(int argc, char **argv)
{
  ros::init("coverage_test", argc, argv);
  
  ros::Rate r(10);
  while(ros::ok()) {
    ROS_DEBUG("Working...");  
    r.sleep();
  }
}
