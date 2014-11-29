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
    nav_msgs::OccupancyGrid& map = map_response.response.map;
    
    // Redefine set of points in boost format
    std::vector<point> L;
    for(int i = 0; i < req.locations.size(); i++) L.push_back( point( req.locations[i].x, req.locations[i].y ) );

    voronoi_diagram<double> vd;
    construct_voronoi( L.begin(), L.end(), &vd );

    std::vector< std::vector<segment> > tessellations;
    const rectangle r = construct<rectangle>( 0, 0, int(map.info.width), int(map.info.height) );
    for(voronoi_diagram<double>::const_cell_iterator it = vd.cells().begin(); it != vd.cells().end(); it++) {
      const voronoi_cell<double> &c = *(it);
      tessellations.push_back( generate_clipped_edges( c, r, L ) );
    }

    // Iterate over each tesselation and compute centroids
    // Use the generate_iteration_range functionality
    res.centroids.resize( tessellations.size() );
    for(int i = 0; i < tessellations.size(); i++) {
      point_data<double> centroid_d(0.,0.);
      double summation = 0;
      double local_density;
      for(int j = 0; j < int(map.info.height); j++) {
        std::pair<int,int> x_range = generate_iteration_range(j, tessellations[i]);
        bool scanning = true;
        if(x_range.first > -1 && x_range.second >= -1) {
          // Use range to iterate over map
          // <= ????
          if(scanning)
            scanning = false;
          for(int x_index = x_range.first; x_index <= x_range.second; x_index++) {
            // Handle Cases:
            //  Black -- Occupied
            //  Gray  -- Unknown
            //  White -- Free
            if( map.data[ j*map.info.width + x_index ] == 0 ||
                map.data[ j*map.info.width + x_index ] == 255 )
              local_density = 0.;
            else
              local_density = 1.;

            centroid_d.x( centroid_d.x() + x_index*local_density );
            centroid_d.y( centroid_d.y() + j*local_density );
            summation += local_density;

          }
        }
        else if(!scanning) {
          // End of the tessellation reached
          break;
        }
      }

      if(summation != 0) {
        centroid_d.x( centroid_d.x() / summation );
        centroid_d.y( centroid_d.y() / summation );
      }
      else {
        // The entire area is covered. Retain robot's position.
        centroid_d.x( double(L[i].x()) );
        centroid_d.y( double(L[i].y()) );
      }

      res.centroids[i].x = int(centroid_d.x());
      res.centroids[i].y = int(centroid_d.y());
          
    }

    ROS_INFO("ComputeCentroids service successfully completed.");
    return true;
  }
  else {
    ROS_ERROR("ComputeCentroids service could not be called successfully.");
    return false;
  }
}
