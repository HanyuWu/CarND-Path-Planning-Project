#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <fstream>
#include <iostream>
#include <string>
#include <uWS/uWS.h>
#include <vector>

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
  char flag = 'k'; // keep stands for keep current lane, l for left-turn, r for
                   // right-turn.
  int lane = 1;    // Initialize initial lane  (0, 1, 2).
  double ref_vel = 0.0; // Set reference velocity (miles/h).

  h.onMessage([&ref_vel, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &lane,
               &flag](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          int prev_size = previous_path_x.size();

          double car_current_s = car_s;

          if (prev_size > 0) {
            car_s = end_path_s; // process not very wise, but still good for
                                // this projecty
          }

          bool too_close = false;

          for (int i = 0; i < sensor_fusion.size(); i++) {
            /**
             * car's unique ID, car's x position in map coordinates, car's y
             * position in map coordinates, car's x velocity in m/s, car's y
             * velocity in m/s, car's s position in frenet coordinates, car's d
             * position in frenet coordinates
             */
            float d = sensor_fusion[i][6];
            if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_s = sensor_fusion[i][5];
              double check_car_s_prime =
                  check_car_s +
                  ((double)prev_size * .02 *
                   check_speed); // Perdict where the car will be in the future
              if ((check_car_s > car_current_s) &&
                  ((check_car_s_prime - car_s) < 30)) {
                // Reduce speed.
                too_close = true;
              }
            }
          }
          /**
           *  SET UP TOO-CLOSE Strategy
           */

          if (too_close) {
            ref_vel -= 0.224; // Deal with the sudden deceleration.
          } else if (ref_vel < 49.5) {
            ref_vel += 0.224; // Deal with cold start.
          }

          if (too_close && lane == 1) {
            // If car on the mid lane, we can switch to the left lane to pass.
            for (int i = 0; i < sensor_fusion.size(); i++) {
              float d = sensor_fusion[i][6];
              if (d < 4 && d > 0) {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx * vx + vy * vy);
                double check_car_s = sensor_fusion[i][5];
                double check_car_s_prime =
                    check_car_s + ((double)prev_size * .02 *
                                   check_speed); // Perdict where the car will
                                                 // be in the future.
                flag = 'l'; // decide if the car can make left turn.
                if ((check_car_s < car_current_s) &&
                    (check_car_s_prime - car_s > 0)) {
                  flag = 'k';
                } else if ((check_car_s > car_current_s) &&
                           (check_car_s_prime - car_s < 30)) {
                  flag = 'k';
                } else if ((check_car_s > car_current_s - 30) &&
                           (check_car_s < car_current_s) &&
                           check_speed > ref_vel) {
                  flag = 'k';
                } else if ((check_car_s > car_current_s - 15) &&
                           (check_car_s < car_current_s)) {
                  flag = 'k';
                }
              }
            }
          } else if (too_close && lane == 0) {
            // If car on the mid lane, we can switch to the left lane to pass.
            for (int i = 0; i < sensor_fusion.size(); i++) {
              float d = sensor_fusion[i][6];
              if (d < 8 && d > 4) {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx * vx + vy * vy);
                double check_car_s = sensor_fusion[i][5];
                double check_car_s_prime =
                    check_car_s + ((double)prev_size * .02 *
                                   check_speed); // Perdict where the car will
                                                 // be in the future.
                flag = 'r'; // decide if the car can make left turn.
                if ((check_car_s < car_current_s) &&
                    (check_car_s_prime - car_s > 0)) {
                  flag = 'k';
                } else if ((check_car_s > car_current_s) &&
                           (check_car_s_prime - car_s < 30)) {
                  flag = 'k';
                } else if ((check_car_s > car_current_s - 30) &&
                           (check_car_s < car_current_s) &&
                           check_speed > ref_vel) {
                  flag = 'k';
                } else if ((check_car_s > car_current_s - 15) &&
                           (check_car_s < car_current_s)) {
                  flag = 'k';
                }
              }
            }
          }

          /**
           *  DESIGN LANE CHANGE
           */

          switch (flag) {
          case 'l':
            lane -= 1;
            flag = 'k';
            break;
          case 'r':
            lane += 1;
            flag = 'k';
          default:
            break;
          }

          /**
           *  SET UP THE PATH BELOW
           */

          vector<double>
              ptsx; // path point_xs which are used to set up our spline
          vector<double>
              ptsy; // path point_ys which are used to set up our spline

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if (prev_size < 2) {
            // set up 2 points
            double prev_car_x =
                car_x - cos(car_yaw); // A point behind current car postion
            double prev_car_y =
                car_y - sin(car_yaw); // but in the same tagent line

            ptsx.push_back(prev_car_x);
            ptsy.push_back(prev_car_y);

            ptsx.push_back(car_x);
            ptsy.push_back(car_y);
          }

          else {
            // set up 2 points
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsy.push_back(ref_y_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);
          }

          // set up 3 more way points at some distance
          vector<double> next_wp0 =
              getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 =
              getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 =
              getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); i++) {
            // Do angle transformation
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = (shift_x * cos(ref_yaw) + shift_y * sin(ref_yaw));
            ptsy[i] = (shift_y * cos(ref_yaw) - shift_x * sin(ref_yaw));
          }

          tk::spline s;
          s.set_points(ptsx, ptsy);
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < previous_path_x.size(); i++) {
            // start with all the previous path points from last time
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;
          for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
            double N =
                (target_dist /
                 (0.02 * ref_vel /
                  2.24)); // 0.02 sec to visit next point, and miles/h to m/s
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;
            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to the fixed coordinate
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
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