
vector<Point> generateStraight(Frenet& fmwk, vector<double>& car, const vector<Point>& previous_path, Point& pathend_sd, const vector<vector<double>>& sensor_fusion) {
          /*
           * ----  Make Smooth transition by starting with previous path 
           */
            
            Point car_xy(car[0], car[1]);
            double car_speed = car[2];
            double car_yaw = car[3];
            double car_s = car[4];
            int prev_size = previous_path.size();
            int retain_pts = min(prev_size, max_retain_pts);
            
            double ref_vel = 49.5/2.237 * 0.02;

            // define the actual (x,y) points we will use for the planner
            vector<Point> new_path = previous_path;
            Point pt = car_xy;
            if (retain_pts>2) {
              pt = previous_path[prev_size-1];
              car_yaw = atan2(pt.y - previous_path[prev_size-2].y, pt.x-previous_path[prev_size-2].x);
            }
            for (int i=retain_pts; i<generate_pts; i++) {
              Point inc = Point (ref_vel*cos(car_yaw), ref_vel*sin(car_yaw));
              pt.add(inc);
              new_path.push_back(pt);
            }
            
            cout << "Setting Points: "; for (int i=0;i<generate_pts;i++) if (i<5 || generate_pts-i<4 || (i%10==0)) cout << "(" << new_path[i].x << "," << new_path[i].y << ") "; cout << endl;
            return new_path;
}