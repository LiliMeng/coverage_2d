#include <boost/polygon/voronoi.h>
#include <boost/polygon/point_data.h>
#include <boost/polygon/segment_data.h>
#include <boost/polygon/rectangle_data.h>
#include <iostream>

typedef boost::polygon::segment_data<int> segment;
typedef boost::polygon::point_data<int> point;
typedef boost::polygon::rectangle_data<int> rectangle;

double ccw(const point&, const point&, const point&);
bool intersect(const segment&, const segment&);
boost::polygon::point_data<double> intersection(const segment&, const segment&);
int signum(double a);
double clamp(double v, double lb, double ub);
bool point_on_segment(const point&, const segment&);
std::pair<int, int> generate_iteration_range(int i, std::vector<segment>edges, int direction = VERTICAL);
std::vector<segment> generate_clipped_edges(const boost::polygon::voronoi_cell<double>& vc, const rectangle& bounding_box, const std::vector<point>& points);
void print(const point&);
void print(const segment&);
