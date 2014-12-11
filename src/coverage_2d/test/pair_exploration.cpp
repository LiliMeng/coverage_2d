#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <nav2d_navigator/MoveToPosition2DAction.h>
#include <coverage_2d/ComputeCentroids.h>
#include <string>
#include <iostream>

typedef actionlib::SimpleActionClient<nav2d_navigator::MoveToPosition2DAction> MoveClient;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pair_exploration");
  ros::NodeHandle nh;
  
  ros::Publisher robot_0_goal_pub = nh.advertise<geometry_msgs::Twist>( "robot_0/goal", 10 );
  ros::Publisher robot_1_goal_pub = nh.advertise<geometry_msgs::Twist>( "robot_1/goal", 10 );
  
  // Initialize the centroid server
  ros::ServiceClient centroid_client = nh.serviceClient<coverage_2d::ComputeCentroids>("compute_voronoi_centroids");
  
  ros::NodeHandle robot_0_handle("robot_0");
  ros::NodeHandle robot_1_handle("robot_1");
  MoveClient robot_0_move_client( robot_0_handle, std::string("MoveTo") );
  MoveClient robot_1_move_client( robot_1_handle, std::string("MoveTo") );
  ROS_INFO("Waiting for robot 0's move client.");
  ROS_INFO("Waiting for robot 1's move client.");
  robot_0_move_client.waitForServer();
  robot_1_move_client.waitForServer();
  
  tf::TransformListener robot_transforms;
   
  if(centroid_client.isValid()) {
    robot_transforms.waitForTransform( "/map", "robot_0/base_link", ros::Time(0), ros::Duration(2.) );
    robot_transforms.waitForTransform( "/map", "robot_1/base_link", ros::Time(0), ros::Duration(2.) );
    // Initial conditions
    int iteration_number = 0;
    int max_iterations = 100;
    while(iteration_number < max_iterations) {
      ROS_INFO("Performing coverage iteration step %d.", iteration_number);
      coverage_2d::ComputeCentroids centroid_compute;
      centroid_compute.request.map_width = 8.5;
      centroid_compute.request.map_height = 7.5;
      centroid_compute.request.locations.resize(2);
      
      tf::StampedTransform robot_0_tf;
      tf::StampedTransform robot_1_tf;
      robot_transforms.lookupTransform( "/map", "robot_0/base_link", ros::Time(0), robot_0_tf );
      robot_transforms.lookupTransform( "/map", "robot_1/base_link", ros::Time(0), robot_1_tf );
      centroid_compute.request.locations[0].x = robot_0_tf.getOrigin().x();
      centroid_compute.request.locations[0].y = robot_0_tf.getOrigin().y();
      centroid_compute.request.locations[1].x = robot_1_tf.getOrigin().x();
      centroid_compute.request.locations[1].y = robot_1_tf.getOrigin().y();

      geometry_msgs::Twist robot_0_twist;
      geometry_msgs::Twist robot_1_twist;
      ROS_INFO("Calling the centroid server.");
      if(centroid_client.call(centroid_compute)) {
        ROS_INFO("Successful response received.");
        robot_0_twist.linear.x = centroid_compute.response.centroids[0].x;
        robot_0_twist.linear.y = centroid_compute.response.centroids[0].y;
        robot_1_twist.linear.x = centroid_compute.response.centroids[1].x;
        robot_1_twist.linear.y = centroid_compute.response.centroids[1].y;
      }
      else {
        ROS_ERROR("Could not compute the centroids.");
      }
      
      nav2d_navigator::MoveToPosition2DGoal robot_0_goal;
      robot_0_goal.target_pose.x = robot_0_twist.linear.x;
      robot_0_goal.target_pose.y = robot_0_twist.linear.y;
      robot_0_goal.target_pose.theta = 0;
      robot_0_goal.target_distance = 0.25;
      robot_0_goal.target_angle = 0.1;
      robot_0_move_client.sendGoal(robot_0_goal);
      
      nav2d_navigator::MoveToPosition2DGoal robot_1_goal;
      robot_1_goal.target_pose.x = robot_1_twist.linear.x;
      robot_1_goal.target_pose.y = robot_1_twist.linear.y;
      robot_1_goal.target_pose.theta = 0;
      robot_1_goal.target_distance = 0.25;
      robot_1_goal.target_angle = 0.1;
      robot_1_move_client.sendGoal(robot_1_goal);
      
      robot_0_move_client.waitForResult(ros::Duration(10.));
      robot_1_move_client.waitForResult(ros::Duration(10.));

      robot_0_goal_pub.publish( robot_0_twist );
      robot_1_goal_pub.publish( robot_1_twist );

      ros::spinOnce();
      iteration_number++;
    }
  }
  else {
    ROS_INFO("Could not start up successfully.");
  }
}
