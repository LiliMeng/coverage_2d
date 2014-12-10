#ifndef COVERAGESERVER_H
#define COVERAGESERVER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>
#include <nav2d_navigator/MoveToPosition2DAction.h>
#include <coverage_2d/geometry.h>
#include <coverage_2d/ComputeCentroids.h>

class CoverageServer {
public:
  typedef coverage_2d::ComputeCentroids::Request ComputeCentroidsRequest;
  typedef coverage_2d::ComputeCentroids::Response ComputeCentroidsResponse;

  CoverageServer();

  bool compute_voronoi_centroids(ComputeCentroidsRequest& req, ComputeCentroidsResponse& res);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer coverage_service_;
  ros::ServiceClient map_client_;
};

#endif // COVERAGESERVER_H
