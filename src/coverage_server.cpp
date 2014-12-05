#include <ros/ros.h>
#include <coverage_2d/CoverageServer.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coverage_server");
  ros::NodeHandle nh;

  ROS_INFO("Starting up coverage server...");
  CoverageServer cs;
  ROS_INFO("Successfully started up the coverage server.");

  ros::spin();
}
