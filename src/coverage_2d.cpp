#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <coverage_2d/ComputeCentroids.h>
#include <string>
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coverage_2d");
  ros::NodeHandle nh;

  int N = 1;  // Define the number of robots
  std::vector<ros::Publisher> robot_goal_pubs(N);
  std::vector<geometry_msgs::Twist> robot_twists(N);
  std::vector<std::string> robot_names;
  for(int i = 0; i < N; i++) {
    std::stringstream ss;
    ss << "robot_" << i;
    robot_names.push_back( ss.str() );
    robot_goal_pubs.push_back( nh.advertise<geometry_msgs::Twist>( robot_names[i].c_str(), 10 ) );
  }

  // Initialize the centroid server
  ros::ServiceClient centroid_server = nh.serviceClient<coverage_2d::ComputeCentroids>("compute_voronoi_centroids");
  tf::TransformListener robot_transforms;
  
  if(N > 0 && centroid_server.isValid()) {
    // Initial conditions
    int iteration_number = 0;
    ROS_INFO("Performing coverage iteration step %d.", iteration_number);
    coverage_2d::ComputeCentroids centroid_compute;
    centroid_compute.request.locations.resize(N);
    
    for(int i = 0; i < robot_names.size(); i++) {
      tf::StampedTransform robot_tf;
      robot_transforms.lookupTransform( robot_names[i].c_str(), "/map", ros::Time(0), robot_tf );
      centroid_compute.request.locations[i].x = robot_tf.getOrigin().x();
      centroid_compute.request.locations[i].y = robot_tf.getOrigin().y();
    }

    for(int i = 0; i < robot_names.size(); i++) {
      if(centroid_server.call(centroid_compute)) {
        robot_twists[i].linear.x = centroid_compute.response.centroids[i].x;
        robot_twists[i].linear.y = centroid_compute.response.centroids[i].y;
      }
      else {
        ROS_ERROR("Could not compute the centroids.");
      }
    }
    
    ros::Rate pubr(30);
    while(ros::ok()) { 
      for(int i = 0; i < robot_goal_pubs.size(); i++) {
        robot_goal_pubs[i].publish( robot_twists[i] );
        pubr.sleep();
        ros::spinOnce();
      }
    }
    iteration_number++;
  }
  else {
    ROS_INFO("Could not start up successfully.");
  }
}
