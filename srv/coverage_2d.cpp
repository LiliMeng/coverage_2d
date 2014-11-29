#include <ros/ros.h>
#include <coverage_2d/CoverageServer.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coverage_2d");
  ros::NodeHandle nh;

  CoverageServer c_server;

  ros::spin();

  return 0;
  
}


