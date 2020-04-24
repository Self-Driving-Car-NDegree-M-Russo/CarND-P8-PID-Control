#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "json.hpp"
#include "PID.h"

#include <boost/log/trivial.hpp>
#include <boost/log/common.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/utility/setup/from_stream.hpp>

namespace logging = boost::log;
namespace attrs = boost::log::attributes;

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // INITIALIZE LOGGER

  // Open the log settings file
  std::string logFileSettingsName = "../logs/settings.txt";
  std::ifstream settings(logFileSettingsName);
  if (!settings.is_open())
  {
    std::cerr << "Could not open log settings file: " << logFileSettingsName << std::endl;
    return -1;
  }

  // Read the settings and initialize logging library
  logging::init_from_stream(settings);

  // Add some attributes
  logging::core::get()->add_global_attribute("TimeStamp", attrs::local_clock()); // each log line gets a timestamp
  logging::core::get()->add_global_attribute("LineID", attrs::counter<unsigned int>(1)); // lines are sequentially numbered

  // LOGGER INITIALIZED

  // Instantiate PID object
  PID pid;

  // Set PID gains
  double Kp = 0.1;      // Initial value for Kp
  double Ki = 0.001;    // Initial value for Ki
  double Kd = 2.8;      // Initial value for Kd

  // User input section
  std::cout << "====================" << std::endl;
  std::cout << "Initial values for PID gains - Kp = " << Kp << "; Ki = " << Ki << "; Kd = " << Kd << std::endl;
  std::cout << "====================" << std::endl;
  //Set tuning flag
  bool do_tune = false;
  string do_tune_in;
  std::cout <<"Do you want to enable tuning with Coordinated Ascent (Twiddle) method [y/(n)]? ";
  getline(std::cin, do_tune_in);
  if ((do_tune_in.compare("Y") == 0) || (do_tune_in.compare("y") == 0)){
    std::cout << "Tuning enabled" <<std::endl;
    do_tune = true;
  }
  else {
    std::cout << "Tuning NOT enabled" <<std::endl;
  }
  std::cout << "====================" << std::endl;

  // Initialize PID
  pid.Init(Kp, Ki, Kd, do_tune);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;

          // Calculate Steering angle
          pid.UpdateError(cte);
          steer_value = pid.OutputSteeringAngle();

          // DEBUG
          BOOST_LOG_TRIVIAL(debug) << "CTE: " << cte << " Steering Value: " << steer_value;

          // If tuning flag active, call tuning algorithm
          if (pid.GetTuneFlag()){
            pid.TuneGains();
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          // DEBUG
          BOOST_LOG_TRIVIAL(debug) << msg;

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
      BOOST_LOG_TRIVIAL(info) << "Connected!!!";
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
      BOOST_LOG_TRIVIAL(info) << "Disconnected";
  });

  int port = 4567;
  if (h.listen(port)) {
    BOOST_LOG_TRIVIAL(info) << "Listening to port " << port;
  } else {
    BOOST_LOG_TRIVIAL(error) << "Failed to listen to port";
    return -1;
  }
  
  h.run();
}