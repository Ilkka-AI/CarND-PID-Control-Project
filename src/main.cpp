#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"
#include <math.h>
using namespace std;
// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;
  

 
 PID pid;
  
  // Initialize the pid variable.

 	pid.Init(0.3,0.01,10);
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
	//just get rid of the annoying warning when compiling  
	double angle_use=angle;
	angle_use+=1;
	 
	  
          /*
          * Calcuate steering value here, the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!          
*/
	  // Update the current, diff and integral errors
	  pid.UpdateError(cte); 	
    	  //cte_diff=(cte-pid.cte_prev);
	  //steer_value=-cte*0.03-cte_diff*0.005;//-pid.cte_integral*0.03;
	  // From Forums, this works Kp = 0.3, Ki = 0.0005, and Kd = 20.
	  //steer_value=-cte*0.3-cte_diff*30-pid.cte_integral*0.0005;

	  // Calculate steer value
          steer_value=-cte*pid.Kp-pid.d_error*pid.Kd-pid.cte_integral*pid.Ki;

	 // For the submission use constant throttle
	 double throttle=0.5;
	 // Some other controls that I tried
	 //double throttle=0.4-(speed-30)*0.005;//-abs(cte_diff)*0.01;
         //cout << "cte " << cte <<" cte_prev "<< pid.cte_prev <<" cte diff " << cte_diff<< " speed " <<speed << endl;  
	 //double throttle=0.3-speedabs(cte)*0.05;//-abs(cte_diff)*0.01;
	 //double throttle=1.3-(abs(angle))*0.02-abs(cte_diff)*0.01-abs(cte)*0.2;	

	 // Update total error		
	 pid.TotalError(); 
          
	 // Send steering commands to the simulator
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	// Update iterations
   	pid.twiddle1.iterations+=1;	

	int n_frames=2200;
	// If the car gets stuck, stop driving and restart immediately to make Twiddling a bit faster.
	// Then add the total error 4.5 squared for the remaining frames. 4.5 is the distance from center to the lane edge.
	// Therefore a car driving out in the middle has smaller total error than one driving out early. 
	if(pid.twiddle1.iterations>2 && speed<0.5){
	pid.total_error_sum+=(n_frames-pid.twiddle1.iterations)*4.5*4.5;
	pid.twiddle1.iterations=(n_frames+1);
}
	// After approx. one lap, check total error and use Twiddle to update parameters 
	if(pid.twiddle1.iterations>n_frames){
 	
	cout << "total error " << pid.total_error_sum/pid.twiddle1.iterations << " Best error "<< pid.twiddle1.best_total_error_per_frame << 		endl;
	cout <<"That error was obtained with" <<endl;
	cout << "pid.Kp " << pid.Kp <<" pid.Ki "<< pid.Ki <<" pid.Kd " << pid.Kd << " speed " <<speed << endl;  	
	// This is the reset message to reset the simulator
	std::string reset_msg = "42[\"reset\",{}]";
		
	ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
	// Update parameters by Twiddle. The error is in units   error/frame	
	pid.twiddle1.TwiddleUpdate(pid.total_error_sum/pid.twiddle1.iterations);
	// Give updated parameters to the PID
	pid.UpdateCoefs();

	cout <<"Now trying" <<endl;
	cout << "pid.Kp " << pid.Kp <<" pid.Ki "<< pid.Ki <<" pid.Kd " << pid.Kd << " speed " <<speed << endl;  		

}
////////////////////////////////////////////////
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
