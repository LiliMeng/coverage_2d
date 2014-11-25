#ifndef COMPUTECENTROIDS_H
#define COMPUTECENTROIDS_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
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
  ros::ServerService coverage_service_;
  ros::ServerClient map_client_;
};

#endif // COMPUTECENTROIDS_H
