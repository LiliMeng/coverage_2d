#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

typedef unsigned char uchar;

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_validation");
  ros::NodeHandle nh;
  
  string p_path = ros::package::getPath("coverage_2d");
  stringstream ss;
  ss << p_path << "/config/test_map.pgm";
  Mat map = imread(ss.str().c_str(), CV_LOAD_IMAGE_GRAYSCALE);

  double c_x = 0., c_y = 0.;
  double summation = 0;
  for(int i = 0; i < map.rows; i++) {
    uchar *p = map.ptr<uchar>(i);
    for(int j = 0; j < map.cols; j++) {
      if(p[j] != 0 && p[j] != 255) {
        c_x += double(i);
        c_y += double(j);
        summation += 1.;
      }
    }
  }
  c_x /= summation;
  c_y /= summation;

  cout << "Image Size: " << map.rows << " x " << map.cols << endl;
  cout << "Centroid: (" << c_x << "," << c_y << ")" << endl;
  return 0;
}
