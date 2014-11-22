#include <ros/ros.h>
#include <boost/polygon/voronoi.hpp>
#include <boost/polygon/point_data.hpp>
#include <boost/polygon/segment_data.hpp>
#include <boost/polygon/rectangle_data.hpp>
#include <iostream>

using namespace boost::polygon;
typedef segment_data<int> segment;
typedef point_data<int> point;

double determinant(const point&, const point&, const point&);
bool intersect(const segment&, const segment&);
point_data<double> intersection(const segment&, const segment&);
void print(const segment&);
void print(const point&);

void print(const point& p)
{
  std::cout << "(" << p.x() << "," << p.y() << ")";
}

void print(const segment& s)
{
  std::cout << "{";
  print(s.low());
  std::cout << ",";
  print(s.high());
  std::cout << "}";
}

point_data<double> intersection(const segment& s0, const segment& s1)
{
  double denom =  (s0.low().x() - s0.high().x())*(s0.low().y() - s0.high().y()) - 
                  (s0.low().y() - s0.high().y())*(s1.low().x() - s1.high().x());
  double v1 = (s0.low().x()*s0.high().y() - s0.low().y()*s0.high().x());
  double v2 = (s1.low().x()*s1.high().y() - s1.low().y()*s1.high().y());
  double px = v1*(s1.low().x() - s1.high().x()) - (s0.low().x() - s0.high().x())*v2;
  double py = v1*(s1.low().y() - s1.high().y()) - (s0.low().y() - s0.high().y())*v2;
  
  double e = 2^-52;
  if(std::abs(denom) > e)
    return point_data<double>(px/denom, py/denom); 
  else
    return point_data<double>();
}

double determinant(const point& p0, const point& p1, const point& p2)
{
  return  (p0.x() * p1.y() + p0.y() * p2.x() + p1.x() * p2.y()) - 
          (p0.x() * p2.y() + p0.y() * p1.x() + p1.y() * p2.x());
}

int signum(double a)
{
  double e = 2^-52;
  if(a > e) return 1;
  if(a < e) return -1;
  return 0;
}

double clamp(double v, double lb, double ub)
{
  if(v < lb) return lb;
  if(v > ub) return ub;
  return v;
}

// Segment intersection
bool intersect(const segment& s0, const segment& s1) 
{
  if( signum(determinant(s0.low(), s0.high(), s1.low())) != 
      signum(determinant(s0.low(), s0.high(), s1.high())) )
    return true;
  else
    return false;
}

// Either a vertical or horizonal segment
bool point_on_segment(const point& p0, const segment& s0)
{
  if(s0.low().x() - s0.high().x() == 0) {
    int y_max = (s0.low().y() > s0.high().y()) ? s0.low().y(): s0.high().y();
    int y_min = (s0.low().y() < s0.high().y()) ? s0.low().y(): s0.high().y();
    if(p0.y() <= y_max && p0.y() >= y_min)
      return true;
    else
      return false;
  }
  else {
    int x_max = (s0.low().x() > s0.high().x()) ? s0.low().x(): s0.high().x();
    int x_min = (s0.low().x() < s0.high().x()) ? s0.low().x(): s0.high().x();
    if(p0.x() <= x_max && p0.x() >= x_min)
      return true;
    else
      return false;
  }
}

std::vector<segment> generate_clipped_edges(const voronoi_cell<double>& vc, const rectangle_data<int>& bounding_box, const std::vector<point>& points)
{
  std::vector<segment> edges;
  
  int box_width = xh(bounding_box) - xl(bounding_box);
  int box_height = yh(bounding_box) - yl(bounding_box);
        
  segment left_bound(ll(bounding_box), ul(bounding_box));
  segment top_bound(ul(bounding_box), ur(bounding_box));
  segment right_bound(ur(bounding_box), lr(bounding_box));
  segment bottom_bound(lr(bounding_box), ll(bounding_box));
  
  // Each cell can only have a maximum of 2 intersections with the boundary.
  std::vector< std::pair<segment, segment> > boundary_intersections;
  
  if(!vc.is_degenerate()) {
    const voronoi_edge<double> *ie = vc.incident_edge();
    do {
      if(ie->vertex0() == NULL && ie->vertex1() == NULL) {
        // Infinite edges on both sides
        std::cout << "Two-sided Infinite" << std::endl;
        segment perpendicular(points[ie->cell()->source_index()], points[ie->twin()->cell()->source_index()]); 
        point_data<double> midpoint(double(perpendicular.low().x()+perpendicular.high().x())/2.0,
                                    double(perpendicular.low().y()+perpendicular.high().y())/2.0);
        double edge_max_length = box_width+box_height;
        double edge_v = double( perpendicular.low().x() - perpendicular.high().x() );
        double edge_h =  double( perpendicular.high().y() - perpendicular.low().y() );
        double edge_unit = pow( pow( edge_v, 2 ) + pow( edge_h, 2 ), 0.5 );
        
        segment max_edge( point(int(midpoint.x() + edge_h * edge_max_length / edge_unit), 
                                int(midpoint.y() + edge_v * edge_max_length / edge_unit)), 
                          point(int(midpoint.x() - edge_h * edge_max_length / edge_unit), 
                                int(midpoint.y() - edge_v * edge_max_length / edge_unit)) );
        
        segment incident_bound;
        std::vector<point> intersection_points;
        
        if(intersect(left_bound, max_edge) && intersection_points.size() < 2) {
          incident_bound = left_bound;
          point p = intersection(left_bound, max_edge);
          intersection_points.push_back( point( int(clamp(p.x(), xl(bounding_box), xh(bounding_box))), 
                                                int(clamp(p.y(), yl(bounding_box), yh(bounding_box))) ) );
        }
        if(intersect(top_bound, max_edge) && intersection_points.size() < 2) {
          incident_bound = top_bound;
          point p = intersection(top_bound, max_edge);
          intersection_points.push_back( point( int(clamp(p.x(), xl(bounding_box), xh(bounding_box))), 
                                                int(clamp(p.y(), yl(bounding_box), yh(bounding_box))) ) );
        }
        if(intersect(right_bound, max_edge) && intersection_points.size() < 2) {
          incident_bound = right_bound;
          point p = intersection(left_bound, max_edge);
          intersection_points.push_back( point( int(clamp(p.x(), xl(bounding_box), xh(bounding_box))), 
                                                int(clamp(p.y(), yl(bounding_box), yh(bounding_box))) ) );
        }
        if(intersect(bottom_bound, max_edge) && intersection_points.size() < 2) {
          incident_bound = bottom_bound;
          point p = intersection(bottom_bound, max_edge);
          intersection_points.push_back( point( int(clamp(p.x(), xl(bounding_box), xh(bounding_box))), 
                                                int(clamp(p.y(), yl(bounding_box), yh(bounding_box))) ) );
        }
        
        if(boundary_intersections.size() == 2) {
          edges.push_back( segment( intersection_points[0], intersection_points[1] ) );
        }
        else {
          // ERROR -- NEED TO HANDLE
          std::cout << "ERROR DOUBLE INFINITE EDGE" << std::endl;
        }
      }
      else if(ie->vertex0() == NULL || ie->vertex1() == NULL) {
        // One-sided infinite
        std::cout << "One-sided Infinite" << std::endl;
        segment perpendicular(points[ie->cell()->source_index()], points[ie->twin()->cell()->source_index()]); 
        
        point midpoint;
        if(ie->vertex0() != NULL) {
          midpoint.x( ie->vertex0()->x() );
          midpoint.y( ie->vertex0()->y() );
        }
        else {
          midpoint.x( ie->vertex1()->x() );
          midpoint.y( ie->vertex1()->y() );
        }
        
        double edge_max_length = box_width+box_height;
        double edge_v = double( perpendicular.low().x() - perpendicular.high().x() );
        double edge_h =  double( perpendicular.high().y() - perpendicular.low().y() );
        double edge_unit = pow( pow( edge_v, 2 ) + pow( edge_h, 2 ), 0.5 );
        
        segment max_edge( point(int(midpoint.x() + edge_h * edge_max_length / edge_unit), 
                                int(midpoint.y() + edge_v * edge_max_length / edge_unit)), 
                          point(int(midpoint.x() - edge_h * edge_max_length / edge_unit), 
                                int(midpoint.y() - edge_v * edge_max_length / edge_unit)) );
        
        bool intersected = true;
        segment incident_bound;
        point intersection_point;
        
        if(intersect(left_bound, max_edge)) {
          incident_bound = left_bound;
          point p = intersection(left_bound, max_edge);
          intersection_point.x( int(clamp(p.x(), xl(bounding_box), xh(bounding_box))) ); 
          intersection_point.y( int(clamp(p.y(), yl(bounding_box), yh(bounding_box))) );
        }
        else if(intersect(top_bound, max_edge)) {
          incident_bound = top_bound;
          point p = intersection(top_bound, max_edge);
          intersection_point.x( int(clamp(p.x(), xl(bounding_box), xh(bounding_box))) ); 
          intersection_point.y( int(clamp(p.y(), yl(bounding_box), yh(bounding_box))) );
        }
        else if(intersect(right_bound, max_edge)) {
          incident_bound = right_bound;
          point p = intersection(left_bound, max_edge);
          intersection_point.x( int(clamp(p.x(), xl(bounding_box), xh(bounding_box))) ); 
          intersection_point.y( int(clamp(p.y(), yl(bounding_box), yh(bounding_box))) );
        }
        else if(intersect(bottom_bound, max_edge)) {
          incident_bound = bottom_bound;
          point p = intersection(bottom_bound, max_edge);
          intersection_point.x( int(clamp(p.x(), xl(bounding_box), xh(bounding_box))) ); 
          intersection_point.y( int(clamp(p.y(), yl(bounding_box), yh(bounding_box))) );
        }
        else {
          intersected = false;
        }
        
        if(intersected) {
          segment boundary_edge;
          if(ie->vertex0() != NULL) {
            boundary_edge.low( midpoint );
            boundary_edge.high( intersection_point );
          }
          else {
            boundary_edge.high( intersection_point );
            boundary_edge.low( midpoint );
          }
          boundary_intersections.push_back( std::pair<segment, segment>( boundary_edge, incident_bound ) );
          edges.push_back( boundary_edge );
        }
        else {
          // ERROR -- NEED TO HANDLE 
          std::cout << "ERROR DOUBLE INFINITE EDGE" << std::endl;
        }
        
      }
      else {
        // Valid segment
        std::cout << "Valid Segment" << std::endl;
        edges.push_back( segment( point( ie->vertex0()->x(), ie->vertex0()->y() ), 
                                  point( ie->vertex1()->x(), ie->vertex1()->y() ) ) );
        
      }
      
      ie = ie->next();
    } while(ie != vc.incident_edge()); 
    
    // Clear out the itnersection points
    if(boundary_intersections.size() == 2) {
      
      point a, b, c;
      point x, y;
      
      if(point_on_segment( boundary_intersections[0].first.low(), boundary_intersections[1].second )) {
        a = boundary_intersections[0].first.high();
        b = boundary_intersections[0].first.low();
      }
      else {
        a = boundary_intersections[0].first.low();
        b = boundary_intersections[0].first.high();
      }
      
      if(point_on_segment( boundary_intersections[1].first.low(), boundary_intersections[1].second )) {
        x = boundary_intersections[1].first.high();
        y = boundary_intersections[1].first.low();
      }
      else {
        x = boundary_intersections[1].first.low();
        y = boundary_intersections[1].first.high();
      }
      
      // Determine direction
      bool clockwise_merge;
      c = boundary_intersections[0].second.low();
      if(signum(determinant( a, b, c )) != signum(determinant( a, b, y )))
        c = boundary_intersections[0].second.high();
      
      if(determinant( a, b, c ) < 0)
        clockwise_merge = true;
      else
        clockwise_merge = false;
      
      // Define the boundary segments in traversal order
      std::vector<segment> boundaries;
      boundaries.push_back( top_bound );
      boundaries.push_back( right_bound );
      boundaries.push_back( bottom_bound );
      boundaries.push_back( left_bound );
      if(clockwise_merge) {
        int b_shift = 0;
        if( boundary_intersections[0].second == right_bound )
          b_shift = 1;
        else if( boundary_intersections[0].second == bottom_bound )
          b_shift = 2;
        else if( boundary_intersections[0].second == left_bound )
          b_shift = 3;
        std::rotate( boundaries.begin(), boundaries.begin() + b_shift, boundaries.end() );
      }
      else {
       std::reverse( boundaries.begin(), boundaries.end() );  
        int b_shift = 0;
        if( boundary_intersections[0].second == top_bound )
          b_shift = 3;
        else if( boundary_intersections[0].second == right_bound )
          b_shift = 2;
        else if( boundary_intersections[0].second == bottom_bound )
          b_shift = 1;
        std::rotate( boundaries.begin(), boundaries.begin() + b_shift, boundaries.end() );
      }
      
      std::vector<segment> boundary_edges;
      bool merged = false;
      int b_index = 0;
      while(!merged) {
        point b_start, b_end;
        segment aligned_bound;
        
        if(clockwise_merge) {
          if(boundaries[b_index].low().x() > boundaries[b_index].high().x()) {
            b_start = boundaries[b_index].low();
            b_end = boundaries[b_index].high();
          }
          else if(boundaries[b_index].low().x() < boundaries[b_index].high().x()) {
            b_start = boundaries[b_index].high();
            b_end = boundaries[b_index].low();
          }
          else {
            if(boundaries[b_index].low().y() < boundaries[b_index].high().y()) {
              b_start = boundaries[b_index].low();
              b_end = boundaries[b_index].high();
            }
            else if(boundaries[b_index].low().y() > boundaries[b_index].high().y()) {
              b_start = boundaries[b_index].high();
              b_end = boundaries[b_index].low();
            }
            else {
              // Points are equivalent -- HANDLE
            }
          }
        }
        else {
          if(boundaries[b_index].low().x() > boundaries[b_index].high().x()) {
            b_start = boundaries[b_index].high();
            b_end = boundaries[b_index].low();
          }
          else if(boundaries[b_index].low().x() < boundaries[b_index].high().x()) {
            b_start = boundaries[b_index].low();
            b_end = boundaries[b_index].high();
          }
          else {
            if(boundaries[b_index].low().y() < boundaries[b_index].high().y()) {
              b_start = boundaries[b_index].high();
              b_end = boundaries[b_index].low();
            }
            else if(boundaries[b_index].low().y() > boundaries[b_index].high().y()) {
              b_start = boundaries[b_index].low();
              b_end = boundaries[b_index].high();
            }
            else {
              // Points are equivalent -- HANDLE
            }
          }
        }
        
        aligned_bound.low( b_start );
        aligned_bound.high( b_end );

        if(point_on_segment( c, aligned_bound ) && point_on_segment( y, aligned_bound )) {
          // Finalize and merge points
          boundary_edges.push_back( segment( c, y ) );
          merged = true;
        }
        else if(point_on_segment( y, aligned_bound )) {
          // Finalize and complete boundary
          boundary_edges.push_back( segment( aligned_bound.low(), y ) );
          merged = true;
        }
        else if(point_on_segment( c, aligned_bound )) {
          // Begin merging
          boundary_edges.push_back( segment( c, aligned_bound.high() ) );
        }
        else {
          // Insert entire segment
          boundary_edges.push_back( aligned_bound );
        }
        b_index++;
      }
    }
    else {
      // ERROR -- NEED TO HANDLE
    }
  }
  return edges;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"voronoi");
  std::vector< point > points;
  points.push_back(point(50,50));
  points.push_back(point(100,50));
  points.push_back(point(150,50));
  points.push_back(point(50,100));
  points.push_back(point(100,100));
  points.push_back(point(150,100));
  points.push_back(point(50,150));
  points.push_back(point(100,150));
  points.push_back(point(150,150));
  
  voronoi_diagram<double> vd;
  construct_voronoi(points.begin(), points.end(), &vd);

  std::vector<segment> segments;
  std::vector<segment> intersections;
  
  // Iterating through all of the edges
  int result = 0;
  for(voronoi_diagram<double>::const_cell_iterator it = vd.cells().begin(); it != vd.cells().end(); it++) {
    const voronoi_diagram<double>::cell_type &cell = *it;
    const voronoi_diagram<double>::edge_type *edge = cell.incident_edge();
    
    if(edge->vertex0() != NULL) 
      std::cout << "(" << edge->vertex0()->x() << "," << edge->vertex0()->y() << ") ";
    if(edge->vertex1() != NULL)
      std::cout << "(" << edge->vertex1()->x() << "," << edge->vertex1()->y() << ")" << std::endl;
    
    std::cout << "Inserting edge into set" << std::endl;
    if(edge->vertex0() != NULL && edge->vertex1() != NULL)  {
      segments.push_back(segment( point(edge->vertex0()->x(), edge->vertex0()->y()),
                                  point(edge->vertex1()->x(), edge->vertex1()->y())) );
    }
    
    do {
      if(edge->is_primary())
        result++;
      edge = edge->next();
    } while(edge != cell.incident_edge());
  }
  // std::cout << result << std::endl;
  
  // Define sweep line
  segment sweepline(point(0,0), point(200,200));
  for(std::vector<segment>::iterator it = segments.begin(); it != segments.end(); it++) {
    if(intersects(sweepline, *it))
      std::cout << "INTERSECTION" << std::endl;
  }
  
  // Testing rectangle data
  const rectangle_data<int> r = construct< rectangle_data<int> >(0, 0, 255, 255);
  /*for(voronoi_diagram<double>::const_cell_iterator sit = vd.cells().begin(); sit != vd.cells().end(); sit++) {
    std::cout << std::endl << std:: endl << "NEW CELL" << std::endl << std::endl;
    const voronoi_cell<double> &c = *(sit);
    std::vector<segment> edges = generate_clipped_edges(c, r, points);
    for(std::vector<segment>::iterator it = edges.begin(); it != edges.end(); it++) {
      print(*it);
    }
  }*/
  
  for(voronoi_diagram<double>::const_cell_iterator sit = vd.cells().begin(); sit != vd.cells().end(); sit++) {
    std::cout << std::endl << std:: endl << "NEW CELL" << std::endl << std::endl;
    const voronoi_cell<double> &c = *(sit);
    std::vector<segment> edges = generate_clipped_edges(c, r, points);
    for(int i = 0; i < edges.size(); i++) {
      print(edges[i]);
    }
  }
  
}
