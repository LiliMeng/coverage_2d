#include <coverage_2d/CoverageServer.h>

using namespace boost::polygon;

CoverageServer::CoverageServer() :
  nh_()
{
  std::string map_service;
  nh_.param<std::string>( "map_service", map_service, std::string("get_map") );
  map_client_ = nh_.serviceClient<nav_msgs::GetMap>( map_service );

  ROS_INFO("Waiting for map service %s to initalize...", map_service.c_str());
  map_client_.waitForExistence();
  ROS_INFO("Service successfully initialized.");

  coverage_service_ = nh_.advertiseService<ComputeCentroidsRequest, ComputeCentroidsResponse>( "compute_voronoi_centroids", boost::bind( &CoverageServer::compute_voronoi_centroids, this, _1, _2 ) );

}

bool CoverageServer::compute_voronoi_centroids(ComputeCentroidsRequest& req, ComputeCentroidsResponse& res)
{
  // Obtain the map from the server
  nav_msgs::GetMap map_response;
  if(map_client_.call(map_response)) {
    nav_msgs::OccupancyGrid& map = map_response.map;
    
    // Redefine set of points in boost format
    std::vector<point> L;
    for(int i = 0; i < req.locations.size(); i++) L.push_back( point( req.locations[i].x, req.locations[i].y ) );

    voronoi_diagram<double> vd;
    construct_voronoi( L.begin(), L.end(), &vd );

    std::vector< std::vector<segment> > tessellation;
    for(voronoi_diagram<double>::const_cell_iterator it = vd.cells().begin(); it != vd.cells().end(); it++) {
      const voronoi_cell<double> &c = *(it);
      tesselation.push_back( generate_clipped_edges( c, r, points ) );
    }

    // Iterate over each tesselation and compute centroid
    // Use the generate_iteration_range functionality

  }
  else {
    ROS_ERROR("Service %s could not be called successfully.", map_service.c_str());
  }
}
