#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

    // start lane 1
  int lane = 1;
  double ref_vel = 0.0; //start at 0 miles per hour set later to increase gradually at 5 m/2
  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          int prev_size=previous_path_x.size(); //Previous path from the simulator, need to keep it to a minimum so it is not stuck using previous points when there is a change in environment
          
          // If there are points left in previous path, set car_s to previous path's end_path_s
          if (prev_size>0)
          {
            car_s=end_path_s;
          }
          
// Lane identifiers for other cars
bool too_close = false;
bool car_left = false;
bool car_right = false;

// Find ref_v to use, see if car is in lane
for (int i = 0; i < sensor_fusion.size(); i++) {
    // Car is in my lane
    float d = sensor_fusion[i][6];

    // Identify the lane of the car in question
    int car_lane;
    if (d >= 0 && d < 4) {
        car_lane = 0;
    } else if (d >= 4 && d < 8) {
        car_lane = 1;
    } else if (d >= 8 && d <= 12) {
        car_lane = 2;
    } else {
        continue;
    }

    // Check width of lane, in case cars are merging into our lane
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx*vx + vy*vy);
    double check_car_s = sensor_fusion[i][5];

    // If using previous points project an s value outwards in time
    // (To determine What position we will be in in the future)
    // check s values greater than ours and s gap
    check_car_s += ((double)prev_size*0.02*check_speed);

    int gap = 30; // m

      // Identify whether the car is ahead, to the left, or to the right
    if (car_lane == lane) {
        // Another car is ahead
        too_close |= (check_car_s > car_s) && ((check_car_s - car_s) < gap);
    } else if (car_lane - lane == 1) {
        // Another car is to the right
        car_right |= ((car_s - gap) < check_car_s) && ((car_s + gap) > check_car_s);
    } else if (lane - car_lane == 1) {
        // Another car is to the left
        car_left |= ((car_s - gap) < check_car_s) && ((car_s + gap) > check_car_s);
    }
} // for loop checking for all cars sensor fusion
          
   // Behavior planning module to determine car's behavior: slow down or shift lanes left or right
          
          // Modulate the speed to avoid collisions. Change lanes if it is safe to do so (nobody to the side)
double acc = 0.224;
double max_speed = 49.5;
if (too_close) {
    // A car is ahead
    // Decide to shift lanes or slow down
    if (!car_right && lane < 2) {
        // No car to the right AND there is a right lane -> shift right
        lane++;
    } else if (!car_left && lane > 0) {
        // No car to the left AND there is a left lane -> shift left
        lane--;
    } else {
        // Nowhere to shift -> slow down
        ref_vel -= acc;
    }
} else {
    if (lane != 1) {
        // Not in the center lane. Check if it is safe to move back
        if ((lane == 2 && !car_left) || (lane == 0 && !car_right)) {
            // Move back to the center lane
            lane = 1;
        }
    }

    if (ref_vel < max_speed) {
        // No car ahead AND we are below the speed limit -> speed limit
        ref_vel += acc;
    }
}
          
          
          json msgJson;

          //Create a list of widely spaced waypoints at 30 m
          // We will interpolate these waypoints with a spline
          vector <double>ptsx;
          vector<double>ptsy;
          
          //keep a track of reference point(x,y,yaw), either where the car is or at the previous path's endpoint
          double ref_x=car_x;
          double ref_y=car_y;
          double ref_yaw=deg2rad(car_yaw); //convert degree to radians
          
          //if previous size is almost empty use the car as the starting reference
          // The lines below (Lines 109-143) generate a starting reference for the car
          if (prev_size<2)
          {
            //use 2 points that make the car tangent to the path
            // If I am just starting out and have no previous path points
            // I go backwards in time and create previous path along the tangent
            double prev_car_x=car_x-cos(car_yaw);
            double prev_car_y=car_y-sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
            //I have previous path points
            //use the previous path's end point as the starting refernce
          {
            //change the reference element to last element in the previous path
            ref_x=previous_path_x[prev_size-1];
            ref_y=previous_path_y[prev_size-1];
            
            double prev_ref_x=previous_path_x[prev_size-2];
            double prev_ref_y=previous_path_y[prev_size-2];
            // calculate what angle the car was heading in using the last couple of points from previous path
            double ref_yaw=atan2(ref_y-prev_ref_y,ref_x-prev_ref_x);
            //push the two points that make the path tangent to the previous end point
            // push these to the list of previous points
            // push a list of 2 points (2x's and 2 y's)
            ptsx.push_back(prev_ref_x);
            ptsx.push_back(ref_x);
           
            ptsy.push_back(prev_ref_y);
            ptsy.push_back(ref_y);                   
          }
            
          // Now that we have a starting reference
          // In Frenet generate a list of evenly spaced points 30, 60, 90 m apart
          // Pushing 3 more points but instead of them being spaced 0.5 m apart, they are spaced 30 m apart
          // The idea is we rely on a spline function to get a smooth transition rather than getXY that creates sharp turns
          // d=6 since it has to be middle lane, and value of lane=1
          vector <double> next_wp0=getXY(car_s+30,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector <double> next_wp1=getXY(car_s+60,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector <double> next_wp2=getXY(car_s+90,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          // The vectors ptsx and ptsy now have 5 points: 2 previous points and points at 30, 60 and 90 m
          // Transformation to make sure the car or last point of previous path are at 0,0 origin and 0 degrees
          // This transformation is very useful for later
          // Shifts the frame of reference to where the car is heading to make the math much easier
          // Its a shift in rotation
          
          for (int i=0; i<ptsx.size(); i++)
          {
            // shift car reference angle to zero degrees
            double shift_x=ptsx[i]-ref_x;
            double shift_y=ptsy[i]-ref_y;
            
            ptsx[i]=(shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i]=(shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }
          
          // Set ptsx and ptsy to the spline
          // We are dealing with 2 sets of points here
          // Set 1: The five points ptsx and ptsy are added to spline s. spline s is a set of anchor points 
          // Set 2: next_x_vals and next_y_vals are future path planner points
          tk::spline s;
          s.set_points(ptsx,ptsy);
          
           
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          //Start with all of previous point paths from last time
          // Instead of recreating the path every sinle time, add the previous path and work from what you had left from last time
          // Creates a smooth transition
          // Add to future path whatever was left from previous path i.e. if there were 50 points and the car already traveled through 3 of these, there are 47 points left
          
          for (int i = 0; i < previous_path_x.size(); i++) 
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          // Calculate how to break up spline so we travel at our reference velocity
          double target_x=30.0;
          double target_y=s(target_x);
          double target_dist=sqrt(target_x*target_x+target_y*target_y);
          
          double x_add_on=0.0;
          //Fill the rest of path planner after filling previous points
          for (int i=1;i<50-previous_path_x.size();i++) {
            double N=target_dist/(0.02*ref_vel/2.24); //2.24 convert from mph to m/s
            double x_point=x_add_on+target_x/N;
            double y_point=s(x_point);
            
            x_add_on=x_point;
            double x_ref=x_point;
            double y_ref=y_point;
            
            //Shift back from local to global coordinates
            x_point=(x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point=(x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
            
            x_point+=ref_x;
            y_point+=ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
            
          }
            
            
          
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}