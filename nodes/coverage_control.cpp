#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <iostream>

void print_costmap_2d(costmap_2d::Costmap2D* cmap)
{
  if(cmap != NULL) {
    for(int i = 0; i < cmap->getSizeInCellsX(); i++) {
      for(int j = 0; j < cmap->getSizeInCellsY(); j++) {
        std::cout << (int)cmap->getCost(i,j) << " ";
      }
      std::cout << std::endl;
    }
  }
  else {
    std::cout << "Costmap is NULL; could not print.";
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "coverage_control");
  
  // Costmap Setup
  tf::TransformListener tl;
  costmap_2d::Costmap2DROS cmap_handler("willow", tl);
  costmap_2d::Costmap2D *cmap = cmap_handler.getCostmap();
  
  if(cmap != NULL) {
    ROS_INFO("Costmap X dimension: %d", cmap->getSizeInCellsX());
    ROS_INFO("Costmap Y dimension: %d", cmap->getSizeInCellsY());
  } 
  else {
   ROS_INFO("Costmap is NULL");
  }
   
  ros::NodeHandle nh;
  ros::Rate r(1);
  while(ros::ok()) {
    print_costmap_2d(cmap);
    r.sleep();
  }
}
