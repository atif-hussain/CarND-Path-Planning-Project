#include "Point.h"

constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

class Frenet {
  public:
  vector<Point> map;
  
  int ClosestWaypoint(Point p) {
    double closestLen = 100000; //large number
    int closestWaypoint = -1;

    for (int i = 0; i < map.size(); i++) {
      double dist = map[i].distance(p);
      if (dist < closestLen) {
        closestLen = dist;
        closestWaypoint = i;
    }}
    return closestWaypoint;
  }

  int NextWaypoint(Point p) {  //, double theta
    int closestWaypoint = ClosestWaypoint(p);
    Point mapt = map[closestWaypoint];
    Point mapp = map[closestWaypoint?(closestWaypoint-1):(map.size()-1)];

    // heading vector is p to map[closestWaypoint]
    double heading = atan2((mapt.y - p.y), (mapt.x - p.x));
    double theta = atan2((mapt.y - mapp.y), (mapt.x - mapp.x));
    double angle = abs(theta - heading);
    if (angle>pi()) angle -= 2 * pi();

    if (angle > pi() / 2) {
      closestWaypoint++;
      closestWaypoint = closestWaypoint % map.size();
    }
    return closestWaypoint;
  }


  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  Point getFrenet(Point p) { // , double theta
    int next_wp = NextWaypoint(p);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0) prev_wp = map.size() - 1;

    // for simplified calculations, take all points wrt Waypoint[prev_wp]
    double n_x = map[next_wp].x - map[prev_wp].x;  //normal is along the path (not unit normal)
    double n_y = map[next_wp].y - map[prev_wp].y;
    double x_x =            p.x - map[prev_wp].x;
    double y_y =            p.y - map[prev_wp].y;

    // find the projection of x onto n
    double proj_norm = (x_x*n_x + y_y*n_y) / (n_x*n_x + n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++)	{
      frenet_s += map[i].distance(map[i+1]);
    }
    frenet_s += sqrt(proj_x*proj_x + proj_y*proj_y);

    // calculate d value
    double area = x_x * n_y - y_y * n_x; // | 0 0 1 || x_x y_y 1 || n_x n_y 1 |
    double frenet_d = area / sqrt(n_x*n_x + n_y*n_y);  // this way computes d with its sign 

    //DEBUG cout <<"getFrenet: x,y="<<p.x<<","<<p.y<<" pts#"<<prev_wp<<" s="<<frenet_s<<" d="<<frenet_d<<endl;
    return Point(frenet_s, frenet_d);
  }

  // Transform from Frenet s,d coordinates to Cartesian x,y
  Point getXY(Point p) {
    double s = p.x, d = p.y;
    int prev_wp = 0;
    
    while ((prev_wp < (int)(map.size() - 1)) && (s > 0))
    {
      double ds = map[prev_wp].distance(map[prev_wp+1]);
      s -= ds;
      prev_wp++;
    }
    if (s<=0) { s += map[prev_wp].distance(map[prev_wp+1]); prev_wp--; }
    
    int next_wp = (prev_wp + 1) % map.size();

    double heading = atan2((map[next_wp].y - map[prev_wp].y), (map[next_wp].x - map[prev_wp].x));
    
    Point pt = Point(s,-d);
    pt.rotate(heading);
    pt.add(map[prev_wp]);
    //DEBUG cout <<"getXY: s="<<p.x<<" d="<<p.y<<" pts#"<<prev_wp<<" x,y="<<pt.x<<","<<pt.y<<endl;
    return pt;
  }
  
  int NextWaypoint(double s) {
    int prev_wp = 0;
    while ((s > 0)) // && (prev_wp < (int)(map.size() - 1)) 
    {
      double next_wp = (prev_wp+1)%map.size();
      double ds = map[prev_wp].distance(map[prev_wp+1]);
      s -= ds;
      prev_wp = next_wp;
    }
    if (s>0) return 0; else return prev_wp;
  }
};
