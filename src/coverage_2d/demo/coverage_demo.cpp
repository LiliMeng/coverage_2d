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
  ros::init(argc, argv, "single_explorer");
  ros::NodeHandle nh;

  ros::Publisher robot_goal_pub = nh.advertise<geometry_msgs::Twist>( "robot_0/goal", 10 );

  // Initialize the centroid server
  ros::ServiceClient centroid_client = nh.serviceClient<coverage_2d::ComputeCentroids>("compute_voronoi_centroids");
  MoveClient move_client("MoveTo", true);
  ROS_INFO("Waiting for the move client.");
  move_client.waitForServer();

  tf::TransformListener robot_transforms;
  
  if(centroid_client.isValid()) {
    robot_transforms.waitForTransform( "/map", "/base_link", ros::Time(0), ros::Duration(2.) );
    // Initial conditions
    int max_iterations = 100;
    int iteration_number = 0;
    while(iteration_number < max_iterations) {
      ROS_INFO("Performing coverage iteration step %d.", iteration_number);
      coverage_2d::ComputeCentroids centroid_compute;
      centroid_compute.request.locations.resize(1);
      
      tf::StampedTransform robot_tf;
      robot_transforms.lookupTransform( "/map", "/base_link", ros::Time(0), robot_tf );
      centroid_compute.request.locations[0].x = robot_tf.getOrigin().x();
      centroid_compute.request.locations[0].y = robot_tf.getOrigin().y();

      geometry_msgs::Twist robot_twist;
      ROS_INFO("Calling the centroid server.");
      if(centroid_client.call(centroid_compute)) {
        ROS_INFO("Successful response received.");
        robot_twist.linear.x = centroid_compute.response.centroids[0].x;
        robot_twist.linear.y = centroid_compute.response.centroids[0].y;
      }
      else {
        ROS_ERROR("Could not compute the centroids.");
      }
      
      nav2d_navigator::MoveToPosition2DGoal goal;
      goal.target_pose.x = robot_twist.linear.x;
      goal.target_pose.y = robot_twist.linear.y;
      goal.target_pose.theta = 0;
      goal.target_distance = 0.25;
      goal.target_angle = 0.1;
      move_client.sendGoal(goal);
      move_client.waitForResult(ros::Duration(10.));

      robot_goal_pub.publish( robot_twist );
      ros::spinOnce();
      iteration_number++;
    }
  }
  else {
    ROS_INFO("Could not start up successfully.");
  }
}
