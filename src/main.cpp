#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;


#define LANE_WIDTH 4.0
#define DT 0.02
#define SEARCH_DISTANCE 50
#define FRONT_TRACKING_DISTANCE 30.0
#define MIN_FRONT_DISTANCE 25.0
#define MIN_REAR_DISTANCE 20.0
#define MPS_TO_MPH 2.24

#define TIME_IN_LANE_SHIFT_ALLOWED 15
#define MAX_SPEED 48.5
#define SPEED_STEP 0.224
#define SPEED_STEP_UNSAFE 0.5

#define WAYPOINT_SPACING 30
#define MAX_S 6945.554 // The max s value before wrapping around the track back to 0


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
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

// check if a lane change is allowed. To avoid jerk, only
// one lane shift at a time is permitted.
bool isLaneShiftAllowed(int from_lane, int to_lane) {

  bool is_allowed = false;

  if ((from_lane== 0) && (to_lane == 1)) {
    is_allowed = true;
  }
  else if ((from_lane == 1) &&
           ((to_lane == 0) || (to_lane == 2))) {
    is_allowed = true;
  }
  else if ((from_lane == 2) && (to_lane == 1)) {
    is_allowed = true;
  }

  return is_allowed;
}


// get lane number
// (0 - left, 1 - center, 2 - right)
int getLaneID(double d) {

  int lane_id = 2;

  if (d <= LANE_WIDTH) {
    lane_id = 0;
  }
  else if (d <= 2.0*LANE_WIDTH) {
    lane_id = 1;
  }
  else {
    lane_id = 2;
  }

  return lane_id;
}

// difference between two s Frenet coordinates taking into account
// that the circuit is circular and there is a wrapping of the s coordinate
// back to 0 at the end of the track. 
double s_difference(double s_1, double s_2)
{
    double diff = s_1 - s_2;

    if (diff < (-0.5 * MAX_S)) {
        diff += MAX_S;
    }
    else if (diff > 0.5 * MAX_S) {
        diff -= MAX_S;
    }

    return diff;
}

// Compute the score of a lane based on the
// lane velocity and the distance to the
// closest car in the lane. If it not safe to switch to
// a lane, the score is negative, to prohibit the car from
// switching to that lane.
double computeLaneScore(const int lane,
                        const int current_lane,
                        const double car_s,
                        const int prev_path_size,
                        const vector<vector<double>> &sensor_fusion) {

    double front_gap = SEARCH_DISTANCE; // Only vehicles whithin SEARCH_DISTANCE are considered
    double lane_velocity = MAX_SPEED;
    bool lane_safe = true;

    // Loop over all the vehicles detected in sensor fusion
    for (int i = 0; i < sensor_fusion.size(); ++i) {
      double other_car_s = sensor_fusion[i][5];
      float d = sensor_fusion[i][6];

      int other_car_lane = getLaneID(d);      

      // Only vehicles in the lane, for which the score we are computing, are taking into account.
      if (other_car_lane == lane) {

        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double other_car_speed = sqrt(vx*vx + vy*vy);

        /* propagate the position of the vehicle forward at the end of the previous path */
        other_car_s += (double) prev_path_size * DT * other_car_speed; 
        double distance_to_other = s_difference(other_car_s, car_s);

        if (distance_to_other > 0.0) { 
          // the car is in front of the ego car.

          if (distance_to_other < front_gap) {
              /* Update the front gap with the distance to the closest car in this lane */
              front_gap = distance_to_other;

              if (other_car_speed < lane_velocity) {
                // Update the lane velocity with the slowest velocity of the detected cars in this lane
                lane_velocity = other_car_speed; 
              }

            if (distance_to_other < MIN_FRONT_DISTANCE) {
              /* If the front gap is less than the minimum, switching to the lane is not safe */
              lane_safe = false;
            }
          }
        }
        else {
          // the car is behind the ego car (rear distances are negative)
          if (distance_to_other > -MIN_REAR_DISTANCE) {
            /* If the rear gap is less than the minimum, switching to the lane is not safe */
            lane_safe = false;
          }
        }
      }
    }

    // The score for this lane weights the lane speed, the front distance to the closest car in this lane and
    // whether it is the center lane, as that lane provides with more lane-changing options
    double score = 0.71*(lane_velocity / MAX_SPEED) + 0.21*(front_gap / SEARCH_DISTANCE) + 0.08*(lane == 1); 

    // Negative scores for switching to unsafe lanes.
    if ((current_lane != lane) && (!lane_safe)) {
      score = -1.0;
    }

    return score;

}

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
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

  // Start in the center lane
  int current_lane = 1;
  
  // Every execution, increase the time spent in the lane. 
  // Reset it when a shift is made. Check it before shifting lanes
  // to avoid rapid succesive lane shifts*/
  int64_t time_in_lane = 0;

  // reference velocity
  double ref_v = 0.0;

  h.onMessage([&current_lane, &time_in_lane, &ref_v,
               &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](
                     uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          // propagate the position of the ego car forward at the end of the previous path
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          // 1. Behaviour planning

          // Get allowed lanes
          vector<int> allowed_lanes;
          allowed_lanes.push_back(current_lane);
          if ((current_lane - 1) >= 0) { 
            allowed_lanes.push_back(current_lane - 1); 
          }
          if ((current_lane + 1) <= 2) { 
            allowed_lanes.push_back(current_lane + 1);
          }

          // From the allowed lines, choose the one with highest score
          int best_lane = current_lane;
          double max_score = 0.0;
          for (int lane : allowed_lanes) {

            const double score = computeLaneScore(lane, current_lane, car_s, prev_size, sensor_fusion);
            if ((score > 0.0) && (score > max_score)) {
                max_score = score;
                best_lane = lane;
            } 
          }

          /* Behavior for the velocity based on the closest car in the same lane as the ego car */
          bool is_too_close = false; // Indicates whether the velocity must be adjusted
          bool is_distance_unsafe = false; // Indicates whether the velocity must be adjusted more agresively

          double closest_car_s = MAX_S; // distance to the closest car in the same lane
          double front_v = -1.0; /* speed of the closest car in the same lane */

          for (int i = 0; i < sensor_fusion.size(); ++i) {            

            double other_car_s = sensor_fusion[i][5];
            float d = sensor_fusion[i][6];

            int other_car_lane = getLaneID(d);

            if (other_car_lane == current_lane) {

              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double other_car_speed = sqrt(vx*vx + vy*vy);
              
              /* propagate the position of the vehicle forward at the end of the previous path */
              other_car_s += (double) prev_size * DT * other_car_speed; 
              double distance_to_other = s_difference(other_car_s, car_s);

             
              if ((distance_to_other > 0.0) && 
                 (distance_to_other < FRONT_TRACKING_DISTANCE)) {

                is_too_close = true; 
                /* if other car is in the same lane closer than the safe front distance, flag
                  that it is too close in order to decelerate */
                if (distance_to_other < closest_car_s) {
                  // update the closest car in lane
                  closest_car_s = distance_to_other;
                  front_v = other_car_speed;
                }

                if (distance_to_other < MIN_FRONT_DISTANCE*0.5) { 
                  /* distance is unsafe. This can happen when a vehicle pulls in front of us */
                  is_distance_unsafe = true; /* Apply harder brakes */
                }
              }
            }
          }         

          time_in_lane += 1; /* every execution, increase the time spent in the lane.*/

          // Peform lane change if the best lane is other than the current one and it is allowed.
          if (best_lane != current_lane) {
            if ((isLaneShiftAllowed(current_lane, best_lane)) &&
                (time_in_lane > TIME_IN_LANE_SHIFT_ALLOWED)) {

              time_in_lane = 0; // reset the time in the current lane
              current_lane = best_lane; // change lane
               
            }
          }

          // Adjust velocity
          if (is_too_close) {
            if (ref_v >= front_v) { /* Decrease speed until we follow the front car's speed */
              ref_v -= SPEED_STEP;
            }

            if (is_distance_unsafe) {
              ref_v -= SPEED_STEP_UNSAFE; /* Apply harder brakes. This can happen when a vehicle pulls in front of us  */
            }
          }
          else if (ref_v < MAX_SPEED) {
            ref_v += SPEED_STEP;

          }




          // 2. Path generation          

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // Using a Spline interpolator, create a smooth path to achieve the desired trajectory.
          // First, create a list of widely spaced (x,y) knot points, evenly spaced at 30m
          vector<double> kpts_x;
          vector<double> kpts_y;
          
          if (prev_size < 2) {
            // Use two points that make the path tangent to the car.
            // if previous path size is too small, go backwards in time based on cars yaw angle)
            double car_x_prev = car_x-cos(car_yaw);
            double car_y_prev = car_y-sin(car_yaw);

            kpts_x.push_back(car_x_prev);
            kpts_x.push_back(car_x);

            kpts_y.push_back(car_y_prev);
            kpts_y.push_back(car_y);

          } 
          else {
            // use the previous path end point as the starting reference.

            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // use two points that make the path tangent to the previous path end point
            kpts_x.push_back(ref_x_prev);
            kpts_x.push_back(ref_x);

            kpts_y.push_back(ref_y_prev);
            kpts_y.push_back(ref_y);
          }

          // In Frenet, add evenly spaced knot points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s + WAYPOINT_SPACING, ((LANE_WIDTH/2) + (LANE_WIDTH * current_lane)), 
                                          map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + WAYPOINT_SPACING*2, ((LANE_WIDTH/2) + (LANE_WIDTH * current_lane)),
                                          map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + WAYPOINT_SPACING*3, ((LANE_WIDTH/2) + (LANE_WIDTH * current_lane)), 
                                          map_waypoints_s, map_waypoints_x, map_waypoints_y);

          kpts_x.push_back(next_wp0[0]);
          kpts_x.push_back(next_wp1[0]);
          kpts_x.push_back(next_wp2[0]);

          kpts_y.push_back(next_wp0[1]);
          kpts_y.push_back(next_wp1[1]);
          kpts_y.push_back(next_wp2[1]);

          
          // transform the knot points to local car coordinates with the last point of the previous path at the origin
          for (int i = 0; i < kpts_x.size(); ++i) {            
            // Translate the car to the origin and set car reference angle to 0 degrees
            
            double x_t = kpts_x[i]-ref_x;
            double y_t = kpts_y[i]-ref_y;           

            kpts_x[i] = (x_t*cos(0-ref_yaw)-y_t*sin(0-ref_yaw));
            kpts_y[i] = (x_t*sin(0-ref_yaw)+y_t*cos(0-ref_yaw));
          }


          // setup the spline interpolator  
          tk::spline desired_spline;
          desired_spline.set_points(kpts_x, kpts_y);

          // define a path made up of (x,y) points that the car will visit sequentially every .02 seconds.
          // These points will be sent to the simulator.
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // First add the previous path points
          for (int i = 0; i < previous_path_x.size(); ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = desired_spline(target_x);
          double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
          double x_base = 0.0;
          
          for (int i = 1; i <= 50-previous_path_x.size(); ++i) {

            // Calculate spacing between spline points so that the car travels at the desired speed
            double N = (target_dist / (DT * ref_v / MPS_TO_MPH));
            double x_point = x_base + (target_x) / N;
            double y_point = desired_spline(x_point);

            x_base = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to global coordinates
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

            // translate to global origin
            x_point += ref_x;
            y_point += ref_y;

            // add the points for the simulator
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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

