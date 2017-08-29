
void anchorSpline(tk::spline& s, const vector<Point>& anchors) {
  vector<double> X, Y;
  for (auto a:anchors) {
    X.push_back(a.x); Y.push_back(a.y);
  }
  for(int i=1;i<X.size();i++) 
    if (X[i]<=X[i-1]) cout << anchors << endl;
  s.set_points(X,Y);
}
int laneNo (vector<double> car) {
  double d = car[6];
  return floor(d/4);
}

void makeFrenet (Frenet& fmwk, vector<vector<double>>& sensor_fusion) {
    // Convert sensed' vehicles velocity in x,y coords to s,d coodinates
    // (Model: s-vel remains predictable, irrespective of lane change)
    const int MAP_SIZE = fmwk.map.size();
    for (auto& c: sensor_fusion) {
        int j = fmwk.NextWaypoint(Point(c[1],c[2]));
        Point next_wp = fmwk.map[j];
        Point prev_wp = fmwk.map[j?(j-1):MAP_SIZE-1];
        double theta = atan2(next_wp.y - prev_wp.y, next_wp.x - prev_wp.x);
        Point vel(c[3],c[4]);
        vel.rotate(-theta);
        c[3] = vel.x; c[4] = -vel.y; //write back for later use; 
        //cout<<"theta="<<theta <<". " <<c<< " is travelling @ "<< vel <<endl;
    }
}

Point goodLane (const vector<vector<double>>& sensor_fusion, vector<double>& ourcar, double car_s, double timediff) {
    // AND based on lane occupancy of sensed vehicles, find the best lane to take & target vel for our car 
    bool slowdown = false; int freelane; vector<double> tgtSpeed (3,1E6);
    for (auto& c: sensor_fusion) {
        double cur_s_diff = c[5] - ourcar[5];
        double est_s_diff = c[5] + c[3]*timediff - car_s;
        if (cur_s_diff>=-5 && est_s_diff<=20) {
          cout << "found car #"<<c[0]<<" in lane"<<laneNo(c) <<" travelling @ " << Point(c[3],c[4]) << " is " << cur_s_diff <<"m/"<< est_s_diff<<"m ahead." << endl;
          if (tgtSpeed[laneNo(c)]>c[3]/2.237*0.02) tgtSpeed[laneNo(c)]=c[3]/2.237*0.02;
    }}
    //retain lane if lane just changed or this lane is free
    if ((laneNo(ourcar)!=lane) || (tgtSpeed[lane]>=1E6)) return Point(lane, tgtSpeed[lane]);
    if (lane!=1) tgtSpeed[2-lane]=0; // 2-lane jumping not allowed
    cout << "Found cars. lane tgtSpeeds=" << (tgtSpeed) << endl;
    if ((tgtSpeed[1]>=tgtSpeed[0]) && (tgtSpeed[1]>=tgtSpeed[2])) return Point(1, tgtSpeed[1]);
    if (tgtSpeed[2]>=tgtSpeed[0]) return Point(2, tgtSpeed[2]);
    else                          return Point(0, tgtSpeed[0]);
}

vector<Point> generatePath(Frenet& fmwk, vector<double>& ourcar, const vector<Point>& previous_path, Point& pathend_sd, const vector<vector<double>>& sensor_fusion) {
          /*
           * ----  Make Smooth transition by starting with previous path 
           */
            
            Point car_xy(ourcar[1], ourcar[2]);
            double car_yaw = ourcar[4];  // new_path shall be generated, using relative coordinate wrt car_xy, car_yaw
            double car_s = fmwk.getFrenet(car_xy).x;
            cout <<"Did "<< int(car_s) <<"m" <<endl; 
            
            double car_speed = ourcar[3]/2.237 * 0.02; // car_speed and goto_vel are in metres/0.02s
            double goto_vel = 48/2.237 * 0.02;

            int prev_size = previous_path.size();
            int retain_pts = min(prev_size, max_retain_pts);

            // define the actual (x,y) points we will use for the planner
            vector<Point> new_path = previous_path;
            Point pt = car_xy;
            vector<Point> anchorPts;
            if (retain_pts>2) {
              pt = previous_path[prev_size-1];
              car_s = fmwk.getFrenet(pt).x; //=pathend_sd.x
              car_speed = pt.distance(previous_path[prev_size-2]);
              car_yaw = atan2(pt.y - previous_path[prev_size-2].y, pt.x-previous_path[prev_size-2].x);
              //anchorPts.push_back(previous_path[prev_size-2]);
            }
            anchorPts.push_back(Point(pt.x-cos(car_yaw), pt.y-sin(car_yaw)));
            anchorPts.push_back(pt); // cout << "looking at " << car_s << "s ";

            
            //check for proximate cars
            Point sensed=goodLane(sensor_fusion, ourcar, car_s, retain_pts*0.02);
            bool slowdown=false; lane = sensed.x; 
            double tgtSpeed=sensed.y; if (tgtSpeed<1E6) slowdown=true;
            if (tgtSpeed>goto_vel) tgtSpeed=goto_vel;
            
            int w = fmwk.NextWaypoint(car_s+30);
            for (int i=1;i<=3;i++,w++) {
              if (w==fmwk.map.size()) w=0;
              Point d = fmwk.map[w];
              d.add(-fmwk.map[(w!=0)?(w-1):(fmwk.map.size()-1)]);
              Point n = Point(d.y,-d.x);
              double m = (2+4*lane)/ n.distance(Point(0,0));
              d = fmwk.map[w]; d.add(Point(m*n.x, m*n.y));
              //fmwk.map[w] + (2+4*lane) along path-90'
              anchorPts.push_back(d);
            } //cout << anchorPts << endl;
            
            
            //rotate anchorPts to car_coords
            for (auto& ap:anchorPts) {
              ap.add(-pt);
              ap.rotate(-car_yaw);
            }
            
            tk::spline s; anchorSpline(s, anchorPts);
            double last_x = 0;
            double pathlength = anchorPts[1].distance(anchorPts[2])+anchorPts[2].distance(anchorPts[3])+anchorPts[3].distance(anchorPts[4]);
            double f = abs(anchorPts[2].x)/anchorPts[1].distance(anchorPts[2]);
            
            
            //cout << "goto_vel=" << goto_vel << " starting car_speed= " << car_speed; 
            // generate forward path along identified spline with car_speed
            for (int i=retain_pts; i<generate_pts && last_x<pathlength; i++) {
              // set car_speed
              if (slowdown && (car_speed-tgtSpeed)/(generate_pts-retain_pts)>0.002)  car_speed -= 0.003;
              else if (slowdown && car_speed>=tgtSpeed && car_speed>=0.002)   car_speed -= 0.002; //10*0.02*0.02 /2;
              else if (car_speed < goto_vel-0.002) car_speed += 0.002; //cout << ", " << car_speed; 
              
              if (0) {
                Point inc = Point (car_speed*cos(car_yaw), car_speed*sin(car_yaw));
                pt.add(inc);
                new_path.push_back(pt);
              } if (0) {
                car_s += car_speed;
                auto npt = fmwk.getXY(Point(car_s,2+4*lane));
                new_path.push_back(npt);
                cout<<"pt#"<<i<<" s"<<car_s<<" "<<npt.x<<","<<npt.y << endl;
              } if (1) {
                // generate new point car coords' x from previous, y on spline
                last_x += car_speed*f; Point npt(last_x, s(last_x));
                //cout<<"pt#"<<i<<" s"<<last_x<<" "<<npt.x<<","<<npt.y << endl;
                // transform back to global X-Y coords
                npt.rotate(car_yaw); npt.add(pt);
                new_path.push_back(npt);
                //cout<<"pt#"<<i<<" s"<<last_x<<" "<<npt.x<<","<<npt.y << endl;
            }}
            //cout << " final car_speed= " << car_speed << endl; 
            
            //cout << "Setting Points: "; for (int i=0;i<generate_pts;i++) if (i<5 || generate_pts-i<4 || (i%10==0)) cout << "(" << new_path[i].x << "," << new_path[i].y << ") "; cout << endl;
            return new_path;
}
