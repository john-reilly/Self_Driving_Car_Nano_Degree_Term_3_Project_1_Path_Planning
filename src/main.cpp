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
  
  //inserted from Q+A video time 23.49 in video
  //lane number
  int lane = 1;//outside scope? was until I passed it below
  double ref_vel =  0 ; // was 49.5 ;//mph
  double gentle_acceleration = 0.224 ; // this was Q+A provided level
  double jerk_limit_acceleration = 0.300 ;// higher level risky but I am getting caught by lane cut off vehicles and need to slow down faster
  double brake_rate = 0.4 ;// I just picked a higher number than above ....
  double emergency_stop = 0.6;// I just picked an even higher number than above...never used in action see report
  
  

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane,&gentle_acceleration,&jerk_limit_acceleration,&brake_rate, &emergency_stop, & ref_vel]
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
		//Start of student section
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
         // from Q+A second part
          
          
          int prev_size = previous_path_x.size();
          //insertion point for section 2 intially commeted out code to adjust spped/revent colision
          
          if(prev_size > 0)
          {
            car_s = end_path_s ;
          }
          
          //bool too_close = false;// this was from the Q+A video as basic slow down
          bool within_40m = false;// this will be to allow a gentle slow down at long range
          bool within_30m = false;//this is the same distance as the Q+A video for normal slow down
          bool within_15m = false;//this will be for hard brake and will create a "jerk" event but better than a crash I think!
          bool within_5m = false;
          // my state related booleans
          bool left_lane_clear = true ; //set these to true so if one even dection is found lane staus is changed
          bool right_lane_clear = true ;
          bool centre_lane_clear = true ;
          bool another_car_in_my_lane = false ; 
          //end of mine
          //find ref_v to use
          for(int i = 0 ; i < sensor_fusion.size() ; i++)
          {         
            float d = sensor_fusion[i][6];
           
            //check left lane
            if(d < (2 + 4 * 0 + 2) && d >(2+ 4 * 0 -2) )
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy * vy);
              double check_car_s = sensor_fusion[i][5];//as provides for same lane below
              check_car_s += ((double ) prev_size * 0.02 * check_speed) ;
        
              if( ((check_car_s- car_s) < 50)&& ((check_car_s- car_s) > -20)  )//changing from 30 - 30
              {
                   left_lane_clear = false ;
              }
            }
            //check centre land
            if(d < (2 + 4 * 1 + 2) && d >(2+ 4 * 1 -2) )
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy * vy);
              double check_car_s = sensor_fusion[i][5];//as provides for same lane below
              check_car_s += ((double ) prev_size * 0.02 * check_speed) ;
            
              if( ((check_car_s- car_s) < 50) && ((check_car_s- car_s) > -20)  )//was 30 and -30
              {
                   centre_lane_clear = false ;
              }
            }  
            //check right lane
            if(d < (2 + 4 * 2 + 2) && d >(2+ 4 * 2 -2) )
            {  
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy * vy);
              double check_car_s = sensor_fusion[i][5];//as provides for same lane below
              check_car_s += ((double ) prev_size * 0.02 * check_speed) ;
              
              if( ((check_car_s- car_s) < 50) && ((check_car_s- car_s) > -20)  )//was 30 and -30
              {
                   right_lane_clear = false ;
              }
            }
                          
            //from Q+A video
            if(d < (2 + 4 * lane + 2) && d >(2+ 4 * lane -2) )//is the car in the same lane as me
            {
              another_car_in_my_lane = true ; //mine
              
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy * vy);
              double check_car_s = sensor_fusion[i][5];
              
              check_car_s += ((double ) prev_size * 0.02 * check_speed) ; 
              
              //Determine how close car in front is
              if((check_car_s > car_s ) && ((check_car_s- car_s) <= 40)&& ((check_car_s- car_s) > 30))
              {
              	within_40m = true;
              }
              else  if((check_car_s > car_s ) && ((check_car_s- car_s) <= 30)&& ((check_car_s- car_s) > 15))
              {
               within_30m = true;
              }
              else if((check_car_s > car_s ) && ((check_car_s- car_s) <= 15)&& ((check_car_s- car_s) > 5))
              {
              	within_15m = true; 
              }
              else if((check_car_s > car_s ) && ((check_car_s- car_s) <= 5)&& ((check_car_s- car_s) > 0))
              {
               within_5m = true;
              }
              
            }
            
          }
// based on above make state decisions here.
//
// DECIOSION LOGIC SECTION
//          
//in English if there is any car any distance ahead in my lane and I am going less than 45 I will choose a free lane (this is for when the cars slows a little and matches the speed of the car ahead without getting close and triggering a lane change) or else if the car ahead is within 30m and there is a free lane I will choose the free lane , if all options are free I will default to the centre lane
          if(  (another_car_in_my_lane == true && ref_vel < 45) || (within_30m == true ) )//so it won't change to frequently
          {
            //if you are in lane 0 go into centre
            if(lane == 0 && centre_lane_clear == true )
            { 
              lane = 1 ;
            }else if(lane == 1 && left_lane_clear == true )
            {//if you are in lane 1 go to left or else right    
              lane = 0 ;//this will be a slight bias to left lane as it appear first
            }else if(lane == 1 && right_lane_clear == true )
            {
              lane = 2 ;
            }else if(lane == 2 && centre_lane_clear == true )
            { //if you are in right go centre
              lane = 1 ;
            } 
            
          }
          //default to centre if choice exists...this is to help make progress in traffic
          if(  (left_lane_clear && centre_lane_clear) || (centre_lane_clear && right_lane_clear)  )
          {
            lane = 1; //default to centre if option of centre or outside lanes exists
          }
       
          
          
          // control speed based on how close the car in front is
          if(within_40m)
          {
            ref_vel -= gentle_acceleration;
          }
          else if(within_30m)
          {
            ref_vel -= jerk_limit_acceleration ;
          }
          else if(within_15m)
          {
            ref_vel -= brake_rate ;
          }
          else if(within_5m)
          {
            ref_vel -=  emergency_stop ;
          }
          else if (ref_vel < 49.5 )
          {
           ref_vel += jerk_limit_acceleration ; // old Q+A value 0.224  ;
          }
           
//
// Terminal screen outputs:
//
           //statements about which lane the ego car thinks it is in or about to enter
          std::cout << " The ego car is thinking........"<< std::endl ;
          std::cout << " I am in the" ;
          if(lane==0) std::cout << " left lane." << std::endl  ;
          else if(lane==1) std::cout << " centre lane." << std::endl  ;
          else if(lane==2) std::cout << " right lane." << std::endl  ;
          //statements about what the ego car thinks about the trafic ahead in same lane
          if(another_car_in_my_lane == false) std::cout << "There are no other cars in my lane for at least 40 meters" << std::endl  ;
          else if(within_40m == true) std::cout << "There is a car ahead of me in my lane within 40 metres " << std::endl  ;
          else if(within_30m == true) std::cout << "There is a car ahead of me in my lane within 30 metres " << std::endl  ;
          else if(within_15m == true) std::cout << "There is a car ahead of me in my lane within 15 metres " << std::endl  ;
          else if(within_5m == true) std::cout << "There is a car ahead of me in my lane within 5 metres " << std::endl  ;
          //statements about what the ego car thinks about the trafic in the 3 lanes
          if(left_lane_clear == true) std::cout << "The left lane is clear." << std::endl ;
          if(centre_lane_clear == true) std::cout << "The centre lane is clear." << std::endl ;
          if(right_lane_clear == true) std::cout << "The right lane is clear." << std::endl ;
 
//Generating trajectories using Spline.h and from Q+A video
          
          //below is part 2 of video but before extra commented out collision avoidance section
          //create list of widely spaced xy points
          vector<double>  ptsx ;
          vector<double>  ptsy ;
          //reference x,y,yaw states
          double ref_x = car_x ;
          double ref_y = car_y ;
          double ref_yaw = deg2rad(car_yaw);
          
          //if previous size is almost empty use car as starting reference
          if(prev_size <2)
          {
            //use 2 points that make path tangent to car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          //else use previuos path points
          else
          {
            //redefine ref state as previous path end point
            ref_x = previous_path_x[prev_size -1 ];
            ref_y = previous_path_y[prev_size -1 ];
            
            double ref_x_prev = previous_path_x[prev_size - 2] ;
            double ref_y_prev = previous_path_y[prev_size - 2] ;
            
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            //use 2 point to make path
            ptsx.push_back(ref_x_prev) ;
            ptsx.push_back(ref_x) ;
            
            ptsy.push_back(ref_y_prev) ;
            ptsy.push_back(ref_y) ;
            
           }
            
			 vector<double> next_wp0 = getXY(car_s + 30, 2 + 4 * lane, map_waypoints_s,map_waypoints_x,map_waypoints_y);
            
             vector<double> next_wp1 = getXY(car_s + 60,  2 + 4 * lane, map_waypoints_s,map_waypoints_x,map_waypoints_y);
            
             vector<double> next_wp2 = getXY(car_s + 90,  2 + 4 * lane, map_waypoints_s,map_waypoints_x,map_waypoints_y);
         
            
          
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for(int i =0;  i< ptsx.size();i++)
          {
            //shift for reference
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }
          
          //create a spline
          tk::spline s;
          
          //set xy
          s.set_points(ptsx, ptsy);

          
          
          for(int i = 0; i < previous_path_x.size(); i++)
          { 
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
            
          }
          
          //calculate how to break up spline
          double target_x = 30 ;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)* (target_y));
          
          double x_add_on = 0;
          //fill up the rest of the path planner
          for(int i = 1; i<= 50 - previous_path_x.size();i++)
          {

            double N = (target_dist/( 0.02 * ref_vel / 2.24 ) );
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point) ; 
            
            x_add_on = x_point ; 
            
            double x_ref = x_point ;
            double y_ref = y_point ;
            
            //restore back after rotating earlier
            x_point = (x_ref * cos(ref_yaw)- y_ref*sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw)+ y_ref*cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
 
		// END of Student sections
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