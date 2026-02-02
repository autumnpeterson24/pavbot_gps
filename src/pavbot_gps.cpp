/*
GPS Node for Pavbot
author: Michael Stalford

Subscribes to:
 - Nothing

 Publishes:
  - sensors/gps/lat: Float32 conversion of lat
  - sensors/gps/lng: Float32 conversion of lng
*/
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "gps.h"

// For I/O with the GPS
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <cerrno>
#include <utility>

class PavbotGPS : public rclcpp::Node
{
  private:

  // CLASS MEMBERS ----------------------------
  std::string port; // the port that the nano is connected to
  int baud; //baud rate for serail communication
  int timeout_ms;

  // declare the publishers here
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lat_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lng_pub;

  //delcare all timers here
  rclcpp::TimerBase::SharedPtr timer;
  
  // GPS
  GPS gps;

  // Keeping channels clean
  std::pair<float, float> lastLatLng;

  // Function using in wall timer to update at a constant hertz
  void update() {
    std::pair<float, float> latLng;
    if(gps.readLatLng(latLng)) {
      // Valid
      RCLCPP_INFO(get_logger(), "Valid packet collection");
    } else {
      // No fix
      RCLCPP_INFO(get_logger(), "NO FIX packet collection or failed connection");
    }

    if (lastLatLng.first != latLng.first || lastLatLng.second != latLng.second) {
      // Put results into lat, lng and publish
      std_msgs::msg::Float32 lat;
      std_msgs::msg::Float32 lng;
      lat.data = static_cast<float>(latLng.first);
      lng.data = static_cast<float>(latLng.second);
      lat_pub->publish(lat);
      lng_pub->publish(lng);
    }

    lastLatLng = latLng;
  }

public:
// CONSTRUCTOR -----------------------------
  PavbotGPS() : Node("pavbot_gps") {
    // Parameters -----------------
    port = declare_parameter<std::string>("port", "/dev/ttyUSB0"); // This is if the Arduino is in the USB0 slot it's just a placeholder
    //baud = declare_parameter<int>("baud", 4800);
    timeout_ms = declare_parameter<int>("timeout_ms", 500);

    // Publishers ------------------
    lat_pub = create_publisher<std_msgs::msg::Float32>("/sensors/gps/lat", rclcpp::QoS(1).transient_local() // a queue depth of 1 to keep the latest value :)
    );
    lng_pub = create_publisher<std_msgs::msg::Float32>("/sensors/gps/lng", rclcpp::QoS(1).transient_local() // a queue depth of 1 to keep the latest value :)
    );

    // timers ----------------
    timer = create_wall_timer( std::chrono::milliseconds(500), std::bind(&PavbotGPS::update, this) // periodic timer for constant update
    );

    // Prep
    lastLatLng.first = 0;
    lastLatLng.second = 0;

    // GPS Stuff
    gps.configure(port, B4800, timeout_ms);
    
    if (gps.open()) {
      // Success
      RCLCPP_INFO(get_logger(), "Successful connection to GPS. Awaiting SAT packets...");
    } else {
      // Failure (did you stop gpsd? If not, you may be here)
      // Stop gpsd:         sudo systemctl stop gpsd
      RCLCPP_INFO(get_logger(), "Failure to connect to GPS :: %s", strerror(errno));
    }
  }
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PavbotGPS>());
  rclcpp::shutdown();
  return 0;
}
