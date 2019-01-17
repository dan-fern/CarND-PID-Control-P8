#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "pid.hpp"

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


int main( )
{
    uWS::Hub h;

    // steering and throttle PID controllers
    PID pid_steering, pid_throttle;

    //P: 0.23236, I: 0.00421296, D: 3.486
    pid_steering.Init( 0.262295, 0.0045614, 3.486 );
    //pid_throttle.Init( 0.316731, 0.0000, 0.0226185 );
    /**
    * TODO: Initialize the pid variable.
    */

    h.onMessage( [&pid_steering, &pid_throttle] (
            uWS::WebSocket<uWS::SERVER> ws,
            char *data,
            size_t length,
            uWS::OpCode opCode )
    {
        // "42" at the start of the message means there websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if( length && length > 2 && data[0] == '4' && data[1] == '2' )
        {
            auto s = hasData( string( data ).substr( 0, length ) );

            if( s != "" )
            {
                auto j = json::parse( s );

                string event = j[0].get<string>( );

                if( event == "telemetry" )
                {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<string>());
                    double speed = std::stod(j[1]["speed"].get<string>());
                    double angle = std::stod(j[1]["steering_angle"].get<string>());
                    double steer_value, throttle_value;

                    /**
                    * TODO: Calculate steering value here, remember the steering
                    * value is [-1, 1].
                    * NOTE: Feel free to play around with the throttle and speed
                    *   Maybe use another PID controller to control the speed!
                    */

                    bool twiddle = true;

                    // update error and calculate steer_value at each step
                    pid_steering.UpdateError( cte, twiddle );
                    steer_value = pid_steering.total_pid_value_;

                    // update error and calculate throttle_value at each step
                    //pid_throttle.UpdateError( fabs( cte ), true );
                    throttle_value = 0.250; //- pid_throttle.total_pid_value_;

                    // DEBUG
                    if( !twiddle )
                    {
                        std::cout << "CTE: " << cte
                        << "\tSteering Command: " << steer_value
                        << "\tSteering Angle: " << angle
                        << "\tThrottle: " << throttle_value
                        << "\tSpeed: " << speed
                        << std::endl;
                    }

                    json msgJson;
                    msgJson[ "steering_angle" ] = steer_value;
                    msgJson[ "throttle" ] = throttle_value;
                    //msgJson["throttle"] = (1 - fabs(steer_value)) * 0.5 + 0.2;

                    auto msg = "42[\"steer\"," + msgJson.dump( ) + "]";
                    //std::cout << msg << std::endl;

                    ws.send( msg.data( ), msg.length( ), uWS::OpCode::TEXT );
                }  // end "telemetry" if
            }
            else
            {
                // Manual driving
                string msg = "42[\"manual\",{}]";
                ws.send( msg.data( ), msg.length( ), uWS::OpCode::TEXT );
            }
        }  // end websocket message if
    } ); // end h.onMessage

    h.onConnection( [&h]( uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req )
    {
        std::cout << "Connected!!!" << std::endl;
    } );

    h.onDisconnection( [&h](
            uWS::WebSocket<uWS::SERVER>
            ws, int code,
            char *message,
            size_t length )
    {
        ws.close( );
        std::cout << "Disconnected" << std::endl;
    } );

    int port = 4567;
    if( h.listen( port ) )
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;

        return -1;
    }

    h.run( );
}
