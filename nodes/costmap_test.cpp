#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>

nav_msgs::OccupancyGrid global_costmap;
nav_msgs::OccupancyGrid local_costmap;
nav_msgs::OccupancyGrid map;

void resetGlobalCostmap( const nav_msgs::OccupancyGridConstPtr& gmap )
{
  global_costmap = *gmap;
}

void resetLocalCostmap( const nav_msgs::OccupancyGridConstPtr& gmap )
{
  local_costmap = *gmap;
}

void resetMap( const nav_msgs::OccupancyGridConstPtr& gmap )
{
  map = *gmap;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "costmap_test");
  ros::NodeHandle nh;

  global_costmap = nav_msgs::OccupancyGrid();
  local_costmap = nav_msgs::OccupancyGrid();
  map = nav_msgs::OccupancyGrid();

  ros::Subscriber costmap_subscriber = nh.subscribe( "/move_base/global_costmap/costmap", 2, resetGlobalCostmap );
  ros::Subscriber localmap_subscriber = nh.subscribe( "/move_base/local_costmap/costmap", 2, resetLocalCostmap );
  ros::Subscriber map_subscriber = nh.subscribe( "/map", 2, resetMap );
  
  ros::Rate r(0.5);

  cv::Mat sample = cv::imread("/home/tgdiriba/Pictures/cat8.jpg");

  cv::namedWindow( "global costmap" );
  cv::namedWindow( "map" );
  cv::namedWindow( "tester" );
  cv::imshow( "tester", sample ); 
  while(ros::ok()) {
    ROS_INFO("PRINTING COSTMAP INFO");
    std::cout << global_costmap.info.width << " " << global_costmap.info.height << std::endl;

    cv::Mat gmap = cv::Mat( global_costmap.info.height, global_costmap.info.width, CV_8U, (void*)(&global_costmap.data[0]) ).clone();
    cv::Mat lmap = cv::Mat( local_costmap.info.height, local_costmap.info.width, CV_8U, (void*)(&local_costmap.data[0]) ).clone();
    cv::Mat mmap = cv::Mat( map.info.height, map.info.width, CV_8U, (void*)(&map.data[0]) ).clone();

    uchar* gmap_row = gmap.ptr<uchar>(0);
    for( int i = 0; i < gmap.cols; i++ ) {
      std::cout << (int)gmap_row[i] << " ";
    }
    
    if( gmap.rows > 0 && gmap.cols > 0 && lmap.rows > 0 && lmap.cols > 0 && mmap.rows > 0 && mmap.cols > 0) {
      cv::imshow( "global costmap", gmap );
      cv::imshow( "local costmap", lmap );
      cv::imshow( "map", mmap );
      cv::imwrite( "/home/tgdiriba/Pictures/global_costmap.png", gmap );
      cv::imwrite( "/home/tgdiriba/Pictures/local_costmap.png", lmap );
      cv::imwrite( "/home/tgdiriba/Pictures/map.png", mmap );
    }

    ros::spinOnce();
    r.sleep();
  }

}
