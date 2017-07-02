#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "MPC.h"
#include "json.hpp"

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (unsigned int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Evaluate first derivation of polynomial.
double polyPrimEval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (unsigned int i = 1; i < coeffs.size(); i++) {
    result += i * coeffs[i] * pow(x, i - 1);
  }
  return result;
}
// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


// tranform from world system to vehicle system
// inverse of affine matrix
//source: http://planning.cs.uiuc.edu/node99.html
Eigen::MatrixXd inverseTransform2D(double new_origin_x,double new_origin_y,double new_origin_psi, Eigen::VectorXd pts_x, Eigen::VectorXd pts_y)
{

  Eigen::MatrixXd affineTransform(3,3);
  affineTransform << cos(new_origin_psi),-sin(new_origin_psi),new_origin_x,
                     sin(new_origin_psi),cos(new_origin_psi),new_origin_y,
                     0, 0, 1;
  Eigen::MatrixXd pts(3,pts_x.size());
  pts.row(0) = pts_x;
  pts.row(1) = pts_y;
  pts.row(2) = Eigen::VectorXd::Ones(pts_x.size());

  Eigen::MatrixXd transformed_pts(3,pts_x.size());
  transformed_pts = affineTransform.inverse()*pts;

  return transformed_pts;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          //double psi_unity = j[1]["psi_unity"];
          double v = j[1]["speed"];
          double steering_angle = j[1]["steering_angle"];
          double throttle = j[1]["throttle"];

          int delay = 100; //in ms
          double Lf = 2.67;

          //Convert waypoint to Eigen Vectors
          Eigen::VectorXd pts_x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx.data(), ptsx.size());
          Eigen::VectorXd pts_y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy.data(), ptsy.size());;

          Eigen::MatrixXd transformed_waypoints = inverseTransform2D(px, py, psi, pts_x, pts_y);

          int order_of_poly = 3;
          //fit a polynomial to the above x and y coordinates
          auto coeffs = polyfit(transformed_waypoints.row(0),transformed_waypoints.row(1),order_of_poly);

          // Calculate the cross track error
          double cte = polyeval(coeffs, 0);
          // Calculate the orientation error
          double epsi = -atan(coeffs[1]);//- atan(polyPrimEval(coeffs, 0));

          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          Eigen::VectorXd actuators(2);
          actuators << steering_angle, throttle;

//          Eigen::VectorXd next_state(state.size());
//          //count for delay
//          next_state(0) = state(0) + state(3) * cos(state(2)) * delay;
//          next_state(1) = state(1) + state(3) * sin(state(2)) * delay;
//          next_state(2) = state(2) + state(3) / Lf * actuators(0) * delay;
//          next_state(3) = state(3) + actuators(1) * delay;
//          next_state(4) = polyeval(coeffs, next_state(0)) - next_state(1);
//          next_state(5) = next_state(2) - atan(polyPrimEval(coeffs, next_state(0)));

          auto vars = mpc.Solve(state, coeffs); // without delay
//          auto vars = mpc.Solve(next_state, coeffs);


          reverse(vars.begin(), vars.end());
          double N = vars.back();
          vars.pop_back();
          //double dt = vars.back();
          vars.pop_back();
          vector<double> deltas;

          for (unsigned int t = 0; t < N - 1; t++)
          {
            deltas.push_back(vars.back());
            vars.pop_back();
          }
          vector<double> accs;

          for (unsigned int t = 0; t < N - 1; t++)
          {
            accs.push_back(vars.back());
            vars.pop_back();
          }

          vector<double> xs;

          for (unsigned int t = 0; t < N - 1; t++)
          {
            xs.push_back(vars.back());
            vars.pop_back();
          }
          vector<double> ys;

          for (unsigned int t = 0; t < N - 1; t++)
          {
            ys.push_back(vars.back());
            vars.pop_back();
          }
          /*
          * Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value = -1*deltas[0]/(deg2rad(25)*Lf);
          double throttle_value = accs[0];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          mpc_x_vals = xs;
          mpc_y_vals = ys;

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          for (unsigned int i = 0; i < ptsx.size(); ++i)
          {
            next_x_vals.push_back(transformed_waypoints(0,i));
            next_y_vals.push_back(transformed_waypoints(1,i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
//          this_thread::sleep_for(chrono::milliseconds(delay));
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
