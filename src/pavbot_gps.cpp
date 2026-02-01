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

// For I/O with the GPS
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <cerrno>

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
  int fd; // The port for interfacing (descriptor-style)
  char buf[256]; // The buffer to hold NMEA strings from the GPS

  // Keeping channels clean
  double last_d_lat;
  double last_d_lng;

  // Function using in wall timer to update at a constant hertz
  void update() {
    // Get serial data from GPS and stick into buf
    getSerial();

    // Parse what is in buf and store in d_lat, d_lng
    double d_lat = 0;
    double d_lng = 0;
    parseNMEA(d_lat, d_lng);

    if (last_d_lat != d_lat || last_d_lng != d_lng) {
      // Put results into lat, lng and publish
      std_msgs::msg::Float32 lat;
      std_msgs::msg::Float32 lng;
      lat.data = static_cast<float>(d_lat);
      lng.data = static_cast<float>(d_lng);
      lat_pub->publish(lat);
      lng_pub->publish(lng);
    }

    last_d_lat = d_lat;
    last_d_lng = d_lng;
  }

  void getSerial() {
    int n = read(this->fd, buf, sizeof(buf) - 1);
    if (n > 0) {
      buf[n] = '\0';
    }
  }

  // Parse helpers
  std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    std::stringstream ss(s); // For easier reading (did this for a parser once)
    std::string item;

    while (std::getline(ss, item, delim)) {
      elems.push_back(item);
    }
    return elems;
  }

  double nmeaToDecimal(const std::string &coord, const std::string &hemisphere) {
    if (coord.empty()) return 0.0; // Dud

    double val = std::stod(coord); // Pray string conversion works (on hands and knees)
    int deg = int(val/100); // This is what they tell me to do (?)
    double min = val - deg * 100;
    double dec = deg + (min / 60.0);

    if (hemisphere == "S" || hemisphere == "W") {
      dec = -dec; // Hemishpere correction
    }

    return dec;
  }

  // Parse packet of NMEA data (most important stuff)
  bool parseNMEA(double &lat, double &lng) {
    std::string sentence = buf;

    if (sentence.size() < 6) return false; // Dud

    // Packet may be one of several structures of GPS data
    // Each one is handled in a different if statement below
    // Some don't include lat/lng and are therefore ignored :)
    // The packet structure is the first 6 chars
    // All still give as comma-sep list :)
    if (sentence.substr(0,6) == "$GPRMC") {
      auto fields = split(sentence, ',');
      if (fields.size() < 7) return false; // Incomplete data; untrusted
      if (fields[2] != "A") return false; // Not "A"ctive ("V"oid)
      RCLCPP_INFO(get_logger(), "Parsing non-void GPRMC packet: %s", sentence.c_str()); 
      lat = nmeaToDecimal(fields[3], fields[4]);
      lng = nmeaToDecimal(fields[5], fields[6]);
      return true;
    }

    else if (sentence.substr(0,6) == "$GPGGA") {
      auto fields = split(sentence, ',');
      if (fields.size() < 6) return false; // Incomplete data; untrusted
      int fix = std::stoi(fields[6]);
      if (fix == 0) return false; // 0 ->  NO FIX
      RCLCPP_INFO(get_logger(), "Parsing non-void GPGGA packet: %s", sentence.c_str()); 
      lat = nmeaToDecimal(fields[2], fields[3]);
      lng = nmeaToDecimal(fields[4], fields[5]);
    }
    return false;
  }

public:
// CONSTRUCTOR -----------------------------
  PavbotGPS() : Node("pavbot_gps") {
    // Parameters -----------------
    port = declare_parameter<std::string>("port", "/dev/ttyUSB0"); // This is if the Arduino is in the USB0 slot it's just a placeholder
    baud = declare_parameter<int>("baud", 4800);
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
    last_d_lat = 0;
    last_d_lng = 0;

    // Logging to the screen when the node is created to make sure it is there :)
    RCLCPP_INFO(get_logger(), "GPS node started"); // log that the node started 

    // GPS Prepping!
    // !!! STOP gpsd first if running !!!
    // sudo systemctl stop gpsd
    fd = ::open(port.c_str(), O_RDONLY | O_NOCTTY | O_SYNC);
    if (fd < 0) {
      RCLCPP_INFO(get_logger(), "Failure to connect to GPS :: %s", strerror(errno));
    } else {
      RCLCPP_INFO(get_logger(), "Successful connection to GPS. Awaiting SAT packets...");
    }

    struct termios tty{};
    tcgetattr(fd, &tty);

    cfsetospeed(&tty, B4800);
    cfsetispeed(&tty, B4800);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;

    tcsetattr(fd, TCSANOW, &tty);
  }
};


int main(int argc, char **argv) {
// SPINNING UP THE ROS NODE
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PavbotGPS>());
  rclcpp::shutdown();
  return 0;
}
