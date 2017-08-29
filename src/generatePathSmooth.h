
vector<Point> generatePathSmooth(Frenet& fmwk, vector<double>& car, const vector<Point>& previous_path, Point& pathend_sd, const vector<vector<double>>& sensor_fusion) {
          /*
           * ----  Make Smooth transition by starting with previous path 
           */
            
            Point car_xy(car[0], car[1]);
            double car_speed = car[2];
            double car_yaw = car[3];
            double car_s = car[4];
            int prev_size = previous_path.size();
            int retain_pts = min(prev_size, max_retain_pts);

            if (retain_pts > 0)
            { // overwrite car_s with end of path retained
              car_s = pathend_sd.x;
            }
            
          /* 
          *		Check lane availability for path planning
          */
            bool too_close = false;
            for (int i=0; i< sensor_fusion.size(); i++) {
              //car is in my lane
              float d = sensor_fusion[i][6];
              if (d<(2+4*lane+2) && d<(2+4*lane-2) ) 
              {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx+vy*vy);
                double check_car_s = sensor_fusion[i][5];
                
                check_car_s += ((double)retain_pts*0.02*check_speed); //if using previous points can project s value out
                //check s values greater than mine and s gap
                if ((check_car_s>car_s) && ((check_car_s-car_s)<30) )
                {
                  cout << " found car @" << check_car_s-(retain_pts*0.02*check_speed) << " will move to " << check_car_s << ". Slowing down.. " << endl;
                  // Do some logic here, lower reference velocity so we don't crash into the car infront of us, could
                  // also flag to try to change lanes
                  // ref_vel = 29.5; //mph
                  too_close = true;
    //              if (lane>0) lane--;
                }
              }                
            }
            
            vector<Point> anchor_pts;
            double ref_vel = car_speed;
            cout << "CurrCar@" << car_s << ":" << car_xy.x << "," << car_xy.y << " headed " << car_yaw << " curr_vel=" << car_speed << " retaining pts " << retain_pts << endl;
            
            // Get last two points to get path tangent to create the starting reference for generating new path
            // either we will reference the starting point at where the car is or at the previous paths' end point
            {
              Point prev(0,0);
              
              // if previous size is almost empty, use the car as starting reference
              if (retain_pts < 2) {
                // Use two points that make the path tangent to the car
                prev = Point(car_xy.x - cos(car_yaw), car_xy.y - sin(car_yaw));
              }
              // use the previous path's end point as starting reference, stored as car (as path generated relative to it) 
              else {
                car_xy = previous_path[prev_size-1];
                prev = previous_path[prev_size-2];

                // also compute car direction and velocity after previous path
                car_yaw = atan2(car_xy.y-prev.y, car_xy.x-prev.x);
                ref_vel = car_xy.distance(prev) /.02 *2.237;
              }
              anchor_pts.push_back(prev);
              anchor_pts.push_back(car_xy);
            }


          /*
          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // later we'll interpolate these waypoints with a spline and fill it in with more points that control speed
          */
          
            // In Frenet, add evenly 30m spaced points ahead of the starting reference
            anchor_pts.push_back(fmwk.getXY(Point(car_s+30, 2+4*lane)));
            anchor_pts.push_back(fmwk.getXY(Point(car_s+60, 2+4*lane)));
            anchor_pts.push_back(fmwk.getXY(Point(car_s+90, 2+4*lane)));
            
            cout << " anchored@ " << car_s<<" "<< (2+4*lane); for (auto &apt: anchor_pts) cout<<" ("<<apt.x<<","<<apt.y<<")"; cout<<endl;

            // anchor_pts are in global XY-coords, convert to current car coordinates (ensures spline has x strictly increasing)
            for (auto &apt: anchor_pts) {
              apt.add(-car_xy);
              apt.rotate(-car_yaw);
            }
            cout << " anchor_wrt_Car "; for (auto &apt: anchor_pts) cout<<"("<<apt.x<<","<<apt.y<<") "; cout<<endl;

            
            // create a spline, & set (x,y) points to the spline
            tk::spline s;
            vector<double> anchor_pts_x,anchor_pts_y;
            for (auto &apt: anchor_pts) {
              anchor_pts_x.push_back(apt.x);
              anchor_pts_y.push_back(apt.y);
            }
            /*IFERROR*/for (int i=1; i<anchor_pts_x.size(); i++) if (anchor_pts_x[i-1]>=anchor_pts_x[i]) {for (int j=0; j<anchor_pts_x.size(); j++) cout <<" anchor_pts_x " << anchor_pts_x[j] << " "; cout << endl;}
            s.set_points(anchor_pts_x,anchor_pts_y);
            

            // define the actual (x,y) points we will use for the planner
            vector<Point> new_path = previous_path;
            
            // Calculate how to break up spline points so that we travel at our desired reference velocity
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);
            double x_add_on = 0;
            
            // Fill up the rest of the path planner after filling it up with previous points, here we will always output 50 points
            for (int i=retain_pts; i < generate_pts; i++)
            {
              //find ref_v to use
              if (too_close)
                ref_vel -= 0.02*tgt_acc*2.237;
              else if (ref_vel < tgt_vel)
                ref_vel += 0.02*tgt_acc*2.237;

              //double N = target_dist / (0.02*ref_vel / 2.237); x += tgt_x/N
              x_add_on += target_x / target_dist * (0.02*ref_vel/2.237);

              Point newpt(x_add_on, s(x_add_on));
              //DEBUG if (i < 5 || generate_pts-i<4 || (i%10==0)) cout << "(" << newpt.x << "," << newpt.y << ") "; cout << endl; 

              //rotate back to normal after rotating it earlier
              newpt.rotate(car_yaw);
              newpt.add(car_xy);

              new_path.push_back(newpt);
            }
            Point fpt = fmwk.getFrenet(Point(new_path[1].x,new_path[1].y));
            cout << "starts " << fpt.x <<","<< fpt.y;
            fpt = fmwk.getFrenet(Point(new_path[49].x,new_path[49].y));
            cout << " ends " << fpt.x <<","<< fpt.y << endl;

            cout << "Setting Points: "; for (int i=0;i<generate_pts;i++) if (i<5 || generate_pts-i<4 || (i%10==0)) cout << "(" << new_path[i].x << "," << new_path[i].y << ") "; cout << endl;
            return new_path;
}