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
using std::cout;

enum State {
  keep_lane,
  left_lane,
  right_lane
};

double lane_cost(double target_speed, double lane_speed) {
  
  double cost = 1 - 1/exp(target_speed-lane_speed);
  
  return cost;
}

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
  const double target_speed = 49.0;
  const double speed_step = 1;
  int lane = 1;
  int target_lane = 1;

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
               &map_waypoints_dx,&map_waypoints_dy,&target_speed,&lane,&target_lane,&speed_step]
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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          cout << "car_x: " << car_x << " car_y: " << car_y << std::endl;
          lane = car_d / 4;
          
          double lead_car_speed = std::numeric_limits<double>::max();
          std::map<State, double> states;
          if (lane - 1 >= 0)
            states[left_lane] = 0.0000001;
          if (lane + 1 < 3)
            states[right_lane] = 0.0000001;
          states[keep_lane] = 0;
          
          for (int i = 0; i < sensor_fusion.size(); i++) {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];
            double check_car_d = sensor_fusion[i][6];

            check_car_s += previous_path_x.size() * 0.02 * check_speed;
            int check_lane = check_car_d / 4;
            
            if (check_lane == lane && check_car_s > car_s && check_car_s < car_s + 60) {
              states[keep_lane] = fmax(lane_cost(target_speed, check_speed), states[keep_lane]);
              lead_car_speed = check_speed;
            }
            double speed_ratio = check_speed / car_speed;
            if (check_lane == lane - 1 && check_car_s > car_s - 20 * speed_ratio && check_car_s < car_s + 100) {
              if (check_car_s < car_s + 30.0/speed_ratio) {
                states[left_lane] = 1;
              } else {
                states[left_lane] = fmax(lane_cost(target_speed, check_speed), states[left_lane]);
              }
            }
            if (check_lane == lane + 1 && check_car_s > car_s - 20 * speed_ratio && check_car_s < car_s + 100) {
              if (check_car_s < car_s + 30.0/speed_ratio) {
                states[right_lane] = 1;
              } else {
                states[right_lane] = fmax(lane_cost(target_speed, check_speed), states[right_lane]);
              }
            }
          }
          
          State min_state = keep_lane;
          double min_state_cost = 1;
          for(const auto& state : states) {
            if (state.second < min_state_cost) {
              min_state = state.first;
              min_state_cost = state.second;
            }
          }
        
          switch (min_state) {
            case left_lane:
              target_lane = lane - 1;
              break;
            case right_lane:
              target_lane = lane + 1;
              break;
            case keep_lane:
              if (lead_car_speed < car_speed) {
                car_speed -= fmin(car_speed - lead_car_speed, speed_step) ;
              } else {
                double step = fmin(lead_car_speed - car_speed, speed_step);
                if (car_speed + step < target_speed) {
                  car_speed += step;
                }
              }
              break;
          }

          vector<double> next_x_spline_vals;
          vector<double> next_y_spline_vals;
          vector<double> xy_anchor;
          double prev_x;
          double prev_y;
          double alpha = deg2rad(car_yaw);

          if (previous_path_x.size() < 2) {
            prev_x = car_x - cos(alpha);
            prev_y = car_y - sin(alpha);
          } else {
            prev_x = previous_path_x[previous_path_x.size() - 2];
            prev_y = previous_path_y[previous_path_y.size() - 2];
            car_x = previous_path_x[previous_path_x.size() - 1];
            car_y = previous_path_y[previous_path_y.size() - 1];
          }
          xy_anchor.push_back(car_x);
          xy_anchor.push_back(car_y);
          cout << "anchor_x: " << xy_anchor[0] << " anchor_y: " << xy_anchor[1] << " car_yaw: " << car_yaw << " alpha: " << alpha << std::endl;
          double shift_x = prev_x - xy_anchor[0];
          double shift_y = prev_y - xy_anchor[1];
          double x = shift_x * cos(-alpha) - shift_y * sin(-alpha);
          double y = shift_x * sin(-alpha) + shift_y * cos(-alpha);
          cout << " shift_x: " << shift_x << " shift_y: " << shift_y << " prev_x: " << prev_x << " prev_y: " << prev_y << std::endl;
          next_x_spline_vals.push_back(x);
          next_y_spline_vals.push_back(y);
          next_x_spline_vals.push_back(0);
          next_y_spline_vals.push_back(0);
          for (int i = 1; i < 2; i++) {
            double spline_distance_correction = fmax(car_speed / target_speed, 0.7) ;
            vector<double> xy = getXY(car_s + 40 * i * spline_distance_correction, target_lane * 4 + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            double shift_x = xy[0] - xy_anchor[0];
            double shift_y = xy[1] - xy_anchor[1];
            double x = shift_x * cos(-alpha) - shift_y * sin(-alpha);
            double y = shift_x * sin(-alpha) + shift_y * cos(-alpha);
            next_x_spline_vals.push_back(x);
            next_y_spline_vals.push_back(y);
            
          }
          for (int i = 0; i < next_x_spline_vals.size(); i++) {
            cout << "path_x_sp: " << next_x_spline_vals[i] << " path_y_sp: " << next_y_spline_vals[i] << std::endl;
          }
          tk::spline spline;
          spline.set_points(next_x_spline_vals, next_y_spline_vals, true);
          double x_path = next_x_spline_vals[next_x_spline_vals.size() - 1];
          double y_path = next_y_spline_vals[next_y_spline_vals.size() - 1];
          double x_step = (x_path/sqrt(x_path * x_path + y_path * y_path)) * 0.02 * car_speed / 2.237;
          cout << " x-step: " << x_step << std::endl;
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          int counter = previous_path_x.size();
          for (double x = x_step; (x < x_path) && (counter <= 30); x += x_step) {
            double y = spline(x);
            next_x_vals.push_back(x * cos(alpha) - y * sin(alpha) + xy_anchor[0]);
            next_y_vals.push_back(x * sin(alpha) + y * cos(alpha) + xy_anchor[1]);
            cout << " x: " << x << " y: " << y << " sin(alpha): " << sin(alpha) << " cos(alpha): " << cos(alpha) << std::endl;
            cout << "path_x: " << next_x_vals[next_x_vals.size() - 1] << " path_y: " << next_y_vals[next_y_vals.size() - 1] << std::endl;
            counter++;
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
