#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
//#include "vehicle.h"
//#include "cost.h"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// A class to store the relevant parameters for the vehicles around us 
class Vehicle {
public:
    int id;
    int lane;
    double x;
    double y;
    double s;
    double d;
    double vel_x;
    double vel_y;
    double speed;
    Vehicle(){

    }
    Vehicle(int id, int lane, double x, double y, double s, double d, double vel_x, double vel_y, double speed){
      this->id = id;
      this->lane = lane;
      this->x = x;
      this->y = y;
      this->s = s;
      this->d = d;
      this->vel_x = vel_x;
      this->vel_y = vel_y;
      this->speed = speed; 
    }

    

};

// A function that assigns cost based on the speed of the lane(fastest lane gets the slowest cost).
// The speed of a lane is defined as the speed of the car with maximum speed in that lane given that, the car in question is less than 100m ahead of us and less than 20m behind us 
double lane_speed_cost(int lane, vector<Vehicle>& vehicles, double s){
    double lane_speed = 50/2.24;
    double max_speed = lane_speed;
    for(Vehicle vehicle : vehicles){
        if(vehicle.lane == lane){
            double distance = vehicle.s - s;
            if(distance < 100 && distance > -20){
                if(vehicle.speed < lane_speed){
                    lane_speed = vehicle.speed;
                }
            }
            
        }
    }

    return max_speed - lane_speed;
}

// A function to check the safety of the lane change, given the destination lane and vehicles around us. 
// Here it is assumed that the car in the destination lane has to have a distance of atleast 30m from the car
bool lane_change_safe(int lane, std::vector<Vehicle>& vehicles, double s){
    for(Vehicle vehicle : vehicles){
        if(vehicle.lane == lane){
            double distance = fabs(vehicle.s - s);
            if(distance<30){
                return false;
            }
        }
    }

    return true;
}

// Initial parameter initialization
int lane = 1;
double ref_vel = 0.0;


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
  

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          double MAX_ACC = 10;
          double MAX_JERK = 10;
          double MAX_VEL = 49.5;
          double DISTANCE_AHEAD = 100;
          double DISTANCE_BEHIND = 20;
          double NUM_PATH_POINTS = 50;
          int NUM_LANES = 3;
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
          int prev_size = previous_path_x.size();
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // get current lane
          int current_lane = (int)(car_d/4.0);

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // Extract relevant data from sensor fusion
          // lane of the cars
          // speed of the cars
          // sensor fusion array structure
          // [car ID, car x in map coordinates, car y in map coordinates, car x velocity in m/s, car y velocity in m/s, car s , car d]
          // only store info if the car is relevant ot our decision making
          vector<Vehicle> vehicles;
          for(int i=0; i<sensor_fusion.size(); i++){
            int lane_obs = -1;
            double d = sensor_fusion[i][6];
            double vel_x = sensor_fusion[i][3];
            double vel_y = sensor_fusion[i][4];
            int id = sensor_fusion[i][0];
            // see lane of car based on d value of car
            // if it is not in the 3 lanes continue
            if(d>0 && d<4){
              lane_obs = 0;
            }else if(d>4 && d<8){
              lane_obs = 1;
            }else if(d>8 && d<12){
              lane_obs = 2;
            }else{
              continue;
            }

            // speed of the car is calculated and it is assumed that the heading of the car is correct w.r.t the lane
            double check_speed = sqrt((vel_x * vel_x) + (vel_y * vel_y));

            Vehicle current_car = Vehicle(sensor_fusion[i][0], lane_obs, sensor_fusion[i][1], sensor_fusion[i][2], sensor_fusion[i][5], sensor_fusion[i][6], vel_x, vel_y, check_speed);

            vehicles.push_back(current_car);
          }
          double current_car_s = car_s;
          if(prev_size>0){
            car_s = end_path_s;
          }

          bool too_close = false;
          // Only change lane if the car in front of us is at a unsafe distance(is only possible if it is too slow)
          for(Vehicle vehicle : vehicles){
            if(vehicle.lane == current_lane){// || vehicle.lane == lane){// Predict the location of car in the current lane in the future and check if we are getting too close, if we keep proceeding as we are. A car being too close is defined as 30m away from us or less.
              double check_speed = vehicle.speed;
              double check_car_s = vehicle.s;
              check_car_s += ((double) prev_size * 0.02 * check_speed);
              if ((check_car_s > car_s) && (check_car_s - car_s <30)){
                if(vehicle.s > current_car_s)
                  too_close = true;
              }
            } 
          }


          
          // Calculate costs for each lane and choose the lane with the minimum cost as the fastest lane
          if(too_close){
            int best_lane = lane;
            double current_lane_cost = lane_speed_cost(lane, vehicles, current_car_s);
            for (int i=0; i<NUM_LANES; i++){
              double cost = lane_speed_cost(i, vehicles, current_car_s);
              std::cout<< "COST : " << cost<< std::endl;
              if(cost<current_lane_cost){
                best_lane = i;
                current_lane_cost = cost;
              }
            }

            std::cout<< best_lane << std::endl;
            if(lane == current_lane){ // check if we are not in the middle of a lane change
              if (best_lane > lane){
                if(lane_change_safe(lane+1, vehicles, current_car_s)){ // if lane change is safe, execite it
                  lane = lane + 1;
                }
              }else if (best_lane < lane){
                if(lane_change_safe(lane-1, vehicles, current_car_s)){
                  lane = lane - 1;
                }
              }
              if(lane == current_lane){

                if(ref_vel >0.224)
                  ref_vel -= 0.224;
              }
            }

            
          }else{ // if no lane change is needed, increase velocity in increments till we reach speed limit
            if(ref_vel < MAX_VEL){
              ref_vel += 0.224;
            }
          }

          std::cout<< "REF VEL : " << ref_vel << " LANE : " << lane << std::endl;

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if (prev_size < 2) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

          }

          double lane_center = 2 + 4*lane;
          vector<double> next_wp0 = getXY(car_s + 30, lane_center, map_waypoints_s,
                                          map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, lane_center, map_waypoints_s,
                                          map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, lane_center, map_waypoints_s,
                                          map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); i++) {
            // shift car reference angle to 0 degree
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          // create a spline
          tk::spline s;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // set (x,y) points to the spline
          s.set_points(ptsx, ptsy);

          // start with the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

          double x_add_on = 0;

          // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points

          for (int i = 1; i <= NUM_PATH_POINTS - previous_path_x.size(); i++) {

            double N = (target_dist
                / (0.02 * (ref_vel/2.24)));  // each 0.02 seconds a new point is reached, transform miles per hour to m/s
            double x_point = x_add_on + (target_x) / N;
            double y_point = s(x_point);
            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotating back to normal after rotating it earlier
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            ;

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }
          json msgJson;

          

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */


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
