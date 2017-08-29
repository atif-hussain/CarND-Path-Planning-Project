#include <fstream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
template <typename T> std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
  if ( !v.empty() ) {
    out << '[';
    std::copy (v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  } return out;
}
#include "spline.h"
using namespace std;
#include "Frenet.h"
std::ostream& operator<< (std::ostream& out, const Point& p) {
    out << '(' << p.x << "," << p.y << ")";
  return out;
}

// for convenience
using json = nlohmann::json;

string hasData(string s) {
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

  //start in lane 1
  double lane = 1;
  
  //have a reference velocity to target
  double tgt_vel = 50; //mph
  double tgt_acc = 5.0; //mps2

  const int max_retain_pts = 50;
  const int generate_pts = 50;

#include "generatePathSmooth.h"
#include "generateStraight.h"
#include "generatePath.h"

//motionPlanner mp; 
int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  Frenet fmwk; //vector<Point>  map_waypoints;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x, y, s, d_x, d_y, theta;
    iss >> x >> y >> s >> d_x >> d_y >> theta;
    fmwk.map.push_back(Point(x,y));
  }
  
#if(WIN32)
  h.onMessage([&fmwk] (uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
#else
  h.onMessage([&fmwk] (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
#endif
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    auto sdata = string(data).substr(0, length); 
    cout << std::setprecision(6) << endl; // sdata << endl; 
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = deg2rad(j[1]["yaw"]);
          double car_speed = j[1]["speed"];
          vector<double> car = {-1, car_x, car_y, car_speed, car_yaw, car_s, car_d}; 

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          vector<Point> previous_path; 
          for (int i=0; i< previous_path_x.size(); i++)
            previous_path.push_back(Point(previous_path_x[i],previous_path_y[i]));
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          Point pathend_sd(end_path_s, end_path_d);

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
          
          /* Read in following inputs
           * vector<vector<double>> sensor_fusion, representing surrounding vehicles, each entry of type
           *   {carId=5,x=775.8,y=1441.7,vel_x=0,vel_y=0,s=6661.772,d=-291.7797}
           *   {carId=5,x=775.8,y=1441.7,vel_s=0,vel_d=0,s=6661.772,d=-291.7797}
           * vector<data> current Car representing its {NaN, x,y, speed,yaw, s,d}. Also Point car_xy in short 
           * vector<Point> previous_path x,y; and Point pathend_sd s,d
           * return: vector<Point> new_path
           */
           makeFrenet(fmwk, sensor_fusion);
           vector<Point> new_path = generatePath(fmwk, car, previous_path, pathend_sd, sensor_fusion);
            
          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            // END     

            
            // define the actual (x,y) points we will use for the planner
            vector<double> next_x_vals;
          	vector<double> next_y_vals;
            
            //DEBUG cout << "Setting Points: "; 
            for (int i=0; i < new_path.size(); i++) {
              next_x_vals.push_back(new_path[i].x);
              next_y_vals.push_back(new_path[i].y);
              //DEBUG if (i < 5 || generate_pts-i<4 || (i%10==0)) cout << "(" << next_x_vals[i] << "," << next_y_vals[i] << ") ";
            }
            //cout << "All Points X: \t"; cout << next_x_vals << endl << "All Points Y: \t"; cout << next_y_vals << endl;
            
          	json msgJson;
            
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
#if(WIN32)
			ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
#if(WIN32)
		ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
		ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
	  }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

#if(WIN32)
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
	  std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code,
	  char *message, size_t length) {
	  ws->close();
	  std::cout << "Disconnected" << std::endl;
  });
#else 
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
	  ws.close();
	  std::cout << "Disconnected" << std::endl;
  });
#endif

  int port = 4567;
#if(WIN32)
  if (h.listen("0.0.0.0",port)) {
#else
  if (h.listen(port)) {
#endif
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

