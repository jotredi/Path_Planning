#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include <math.h>
#include <numeric>
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

  // vehicle's lane
  unsigned LANE = 1;
  // state
  string STATE = "KL";
  // speed limit
  double SPEED_LIMIT = 50;

  // velocity at each cycle
  double v = 0;
  // counter for FSM
  unsigned counter = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&LANE,&STATE,&SPEED_LIMIT,
               &v,&counter]
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

          // ====================  PROJECT CODE  ==================== //

          // Reference velocity
          double ref_vel = SPEED_LIMIT - 0.2;

          // Last path size
          unsigned path_size = 0;
          if(previous_path_x.size() >= 5){
            path_size = 5;
          }

          if(path_size){
            // car s at the end of last path
            car_s += path_size*.02*(car_speed/2.237);
          }

          // ============= CRUISE CONTROL ============= //
          bool car_following = false;
          bool very_close = false, very_very_close = false;
          bool about_hit = false;

          for(unsigned i=0; i<sensor_fusion.size(); ++i){
            float d = sensor_fusion[i][6];

            // check if car in our lane
            if(d >= (4*LANE - .5) && d <= (4*LANE + 4.5)){

              // get car's position and velocity
              double s = sensor_fusion[i][5];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double v_car_front = sqrt(pow(vx,2) + pow(vy,2));

              // predict vehicle position at the end of last path
              s += path_size*.02*v_car_front;

              // check if is in front of us
              if(s > car_s){
                // check distance to car
                double dist = s - car_s;

                // distance threshold is proportional to my car speed
                // if we are going faster, we should let more distance
                if(dist < 0.8*car_speed){
                  car_following = true;
                  // take car's velocity as our reference velocity
                  ref_vel = v_car_front * 2.237;
                }

                // check if we are very close to the vehicle
                if(dist < 0.4*car_speed){
                  very_close = true;
                }

                // check if we are very very close
                if(dist < 0.3*car_speed){
                  very_very_close = true;
                  // set reference velocity less than car in front
                  ref_vel = 0.9 * v_car_front * 2.237;
                }

                // check if we are about to hit the vehicle
                if(dist < 0.2*car_speed){
                  about_hit = true;
                }
              }
            }
          }


          // ============== CALCULATE LANE SPEEDS ============== //

          // v_ld: lane velocities lookahed distance
          // if following a vehicle & going slow, dont look too far,
          // if not following a vehicle, look far
          double v_ld;
          if(car_following && car_speed < 0.9*SPEED_LIMIT){
            v_ld = 2*car_speed; // m
            // this distance depends on car speed
          }
          else{
            v_ld = 999; // m
          }

          // Take each vehicle's velocity for each lane
          vector<vector<double>> lanes(3);
          for(unsigned i=0; i<sensor_fusion.size(); ++i){
            double s = sensor_fusion[i][5];
            float d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double v_car = sqrt(pow(vx,2) + pow(vy,2));

            s += path_size*.02*v_car;

            // only take vehicles in front & inside the lookahed distance
            // note: lookahed dist doen't affect my lane
            if(s >= car_s && ((s - car_s) < v_ld ||
              (d >= 4*LANE && d <= (4*LANE + 4)))){
              // left lane
              if(d>=0 && d<=4){
                lanes[0].push_back(v_car);
              }
              // center lane
              else if(d>4 && d<=8){
                lanes[1].push_back(v_car);
              }
              // right lane
              if(d>8 && d<=12){
                lanes[2].push_back(v_car);
              }
            }
          }

          // Set each lane's speed (mph)
          vector<double> lane_v(3);
          for(unsigned i=0; i<lanes.size(); ++i){

            // If no cars, lane velocity is equal to speed limit
            if(lanes[i].empty()){
              lane_v[i] = SPEED_LIMIT;
            }

            // else, we take the minimum speed for each lane (mph)
            // or the maximum speed (if it is my lane)
            else{
              if(i == LANE){
                lane_v[i] = *max_element(lanes[i].begin(), lanes[i].end()) * 2.237;
              }
              else{
                lane_v[i] = *min_element(lanes[i].begin(), lanes[i].end()) * 2.237;
              }
            }
          }

          // Calculate best lane (max velocity)
          double max_v = -99;
          unsigned best_lane;
          for(unsigned l=0; l<lane_v.size(); ++l){
            if(lane_v[l] > max_v){
              max_v = lane_v[l];
              best_lane = l;
            }
          }

          // Only take best lane if velocity increase (no similar speeds)
          if((lane_v[best_lane] - lane_v[LANE]) < 5){
            best_lane = LANE;
          }

          // DEBUG MSGS
          //std::cout << "\nBest lane: " << best_lane << std::endl;
          //std::cout << "My lane: " << LANE << std::endl;
          //std::cout << "Lane velocities: " << std::endl;
          //std::cout << lane_v[0] << std::endl;
          //std::cout << lane_v[1] << std::endl;
          //std::cout << lane_v[2] << std::endl;

          // ================== FINITE STATE MACHINE ================== //

          if(STATE == "KL"){
            // Keep Lane

            if(car_following){
              if(best_lane == LANE){
                STATE = "KL";
              }
              else if(best_lane < LANE && counter > 30){
                STATE = "PLCL";
                counter = 0;
              }
              else if(best_lane > LANE && counter > 30){
                STATE = "PLCR";
                counter = 0;
              }
            }
            // increase counter
            if(counter <= 30)  counter++;
          }
          else if(STATE == "PLCR"){
            // Prepare Lane Change to the Right

            // Adapt velocity to desired lane (if not very close to a vehicle)
            if(!very_close){
              ref_vel = lane_v[best_lane];
            }

            bool free = true;
            for(unsigned i=0; i<sensor_fusion.size(); ++i){
              float d = sensor_fusion[i][6];

              // check if car in right lane
              if(d >= 4*(LANE+1) && d <= (4*(LANE+1) + 4)){
                double s = sensor_fusion[i][5];
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double v_car = sqrt(pow(vx,2) + pow(vy,2));
                s += path_size*.02*v_car;

                // distance that should be free in right lane
                double dist_front = 0.3*car_speed;
                double dist_behind;

                if(s < car_s){
                  // if vehicle behind, leave distance proportional to its vel
                  dist_behind = 0.25*(v_car*2.237); // m
                }
                else{
                  dist_behind = 10; // m
                }
                // check if the car is near (inside threshold distances)
                if((s > (car_s-dist_behind) && s < (car_s+dist_front)) ||
                    very_very_close){
                  // lane change not possible (vehicle in right lane)
                  // or very very close to vehicle in front
                  free = false;
                  // increase counter
                  counter++;
                }
              }
            }
            // Change lanes if right lane is free
            if(free){
              STATE = "LCR";
            }
            // Abort lane change after some time
            if(counter > 100){
              STATE = "KL";
            }
          }
          else if(STATE == "PLCL"){
            // Prepare Lane Change to the Left

            // Adapt velocity to desired lane (if not very close to a vehicle)
            if(!very_close){
              ref_vel = lane_v[best_lane];
            }

            bool free = true;
            for(unsigned i=0; i<sensor_fusion.size(); ++i){
              float d = sensor_fusion[i][6];

              // check if car in left lane
              if((d >= 4*(LANE-1) && d <= (4*(LANE-1) + 4))){
                double s = sensor_fusion[i][5];
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double v_car = sqrt(pow(vx,2) + pow(vy,2));
                s += path_size*.02*v_car;

                // distance that should be free in left lane
                double dist_front = 0.3*car_speed;
                double dist_behind;

                if(s < car_s){
                  // if vehicle behind, leave distance proportional to its vel
                  dist_behind = 0.25*(v_car*2.237); // m
                }
                else{
                  dist_behind = 10; // m
                }
                // check if the car is near (inside threshold distances)
                if((s > (car_s-dist_behind) && s < (car_s+dist_front)) ||
                    very_very_close){
                  // lane change not possible (vehicle in left lane)
                  // or very very close to vehicle in front
                  free = false;
                  // increase counter
                  counter++;
                }
              }
            }
            // Change lanes if left lane is free
            if(free){
              STATE = "LCL";
            }
            // Abort change lane after some time
            if(counter > 100){
              STATE = "KL";
            }
          }
          else if(STATE == "LCR"){
            // Lane Change to the Right
            LANE += 1;
            STATE = "KL";
            counter = 0;
          }
          else if(STATE == "LCL"){
            // Lane Change to the Left
            LANE -= 1;
            STATE = "KL";
            counter = 0;
          }

          // DEBUG MSGS
          //std::cout << "State: " << STATE << std::endl;
          //std::cout << "Counter: "<< counter << std::endl;


          // ============= SET CYCLE SPEED & DISTANCE INCREMENT ============= //
          // Here I set the velocity and distance increment
          // v: velocity of the car in this cycle
          // dist_inc: distance between points in this cycle

          if(car_speed < ref_vel && !about_hit){
            // Increase velocity slowly proportional to how far I am from
            // the reference velocity.
            // (only increase if not very very close to a vehicle)
            v += 0.015*std::fabs(ref_vel - car_speed);
          }
          else if(car_speed > ref_vel && !about_hit){
            if(!very_close){
              // Decrease velocity smoothly if I'm not too close to a car in front
              v -= 0.05*std::fabs(ref_vel - car_speed);
            }
            else{
              // Brake harder if we are close but still not about to hit the vehicle
              v -= 0.1*std::fabs(ref_vel - car_speed);
            }
          }
          else{
            // Set cycle's velocity as the reference velocity
            v = ref_vel;
          }

          // Distance between points
          double dist_inc = (v*0.02)/2.237;


          // ================== TRAJECTORY GENERATION ================== //

          // pts vectors (for generating the spline)
          vector<double> ptsx;
          vector<double> ptsy;

          // ref points (for car transform)
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // Push back current and previous position of the car to ptsx & ptsy
          if(path_size < 2){

            // estimate previous position
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else{

            // Take last points from previous_path vectors
            ref_x = previous_path_x[path_size-1];
            ref_y = previous_path_y[path_size-1];

            double ref_x_prev = previous_path_x[path_size-2];
            double ref_y_prev = previous_path_y[path_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // Select next waypoints
          double ld; // lookahed distance for selecting the waypoints
          if(car_speed < 10){
            ld = 10; // m
          }
          else{
            ld = car_speed; // m
          }
          vector<double> next_wp1 = getXY(car_s + ld, (2+4*LANE), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 1.5*ld, (2+4*LANE), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp3 = getXY(car_s + 2*ld, (2+4*LANE), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          // Push back waypoints for generating the spline
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsx.push_back(next_wp3[0]);

          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          ptsy.push_back(next_wp3[1]);

          // Transform points to the cars coordenate frame
          for (unsigned i=0; i < ptsx.size(); ++i){

            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
          }

          // Set the spline with the points
          tk::spline s;
          s.set_points(ptsx, ptsy);

          // Add last points to the new path
          for(unsigned i=0; i<path_size; ++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Add new points sampling from the spline with the corresponding
          // distance increment
          double x_add = 0;
          for (unsigned i=0; i<45-path_size; ++i){

            // new point (car frame)
            double x_point = x_add + dist_inc;
            double y_point = s(x_point);

            x_add = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // Transform back to global frame
            x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
            y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            // Push back the points to the path
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          // END PROJECT CODE

          //======================================================//


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
