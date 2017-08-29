#ifndef _Point_h
#define _Point_h

//Point stores car's location, or of any point 
//  global x,y location 
//  location wrt car's local -> x ahead of car, and y to its left
//  s,-d coordinates 
class Point {
public:
  double x, y;
  Point(double _x, double _y) {
    x = _x;
    y = _y;
  }
	void rotate(double theta) {
		double _x = x*cos(theta) - y*sin(theta);
		double _y = x*sin(theta) + y*cos(theta);
		x = _x; y = _y;
	}
	void add(const Point p) {
    x += p.x;
    y += p.y;
  }
  // overloaded minus (-) operator
  Point operator- () {
     return Point(-x, -y);
  }
  double distance(Point p) {
    return sqrt((p.x - x)*(p.x - x) + (p.y - y)*(p.y - y));
  }
};

#endif
