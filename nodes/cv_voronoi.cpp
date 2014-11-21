#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cv_voronoi");
  
  std::string impath = "/home/tgdiriba/Code/ros_ws/src/coverage_control/res/images/vtest_1.png";
  cv::Mat image = cv::imread(impath.c_str(), CV_LOAD_IMAGE_COLOR);
  cv::namedWindow("Sample", CV_WINDOW_AUTOSIZE);
  cv::imshow("Sample", image);
  cv::waitKey(0);
}
