// pavbot_gps_node.cpp
//
// Publishes:
//   /gps/fix     (sensor_msgs/msg/NavSatFix)
//   /gps/cog_deg (std_msgs/msg/Float32)   course over ground deg (0=north, 90=east)
//   /gps/sog_mps (std_msgs/msg/Float32)   speed over ground m/s
//
// Notes:
// - Uses $GPRMC primarily for lat/lon + validity + speed/course.
// - Falls back to $GPGGA for lat/lon if needed (but no speed/course there).
// - Implements a line buffer so NMEA parsing works even when reads split lines.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/float32.hpp>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

#include <cmath>
#include <string>
#include <vector>
#include <sstream>
#include <optional>
#include <algorithm>

class GPS {
public:
  GPS(std::string port = "/dev/ttyUSB0", speed_t baud = B4800, int timeout_ms = 500)
  : port_(std::move(port)), baud_(baud), timeout_ms_(timeout_ms) {}

  void configure(const std::string& port, speed_t baud, int timeout_ms) {
    port_ = port;
    baud_ = baud;
    timeout_ms_ = timeout_ms;
  }

  bool open_port() {
    fd_ = ::open(port_.c_str(), O_RDONLY | O_NOCTTY | O_SYNC);
    if (fd_ < 0) return false;

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) return false;

    cfsetospeed(&tty, baud_);
    cfsetispeed(&tty, baud_);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag = 0;
    tty.c_oflag = 0;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = timeout_ms_ / 100; // deciseconds

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) return false;
    return true;
  }

  void close_port() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  }

  // Read raw bytes and append into buffer_
  void pump() {
    if (fd_ < 0) return;
    char buf[256];
    const int n = ::read(fd_, buf, sizeof(buf));
    if (n > 0) buffer_.append(buf, buf + n);
  }

  // Return next complete NMEA line (without \r\n). If none available, return nullopt.
  std::optional<std::string> next_line() {
    // NMEA lines typically end with "\r\n"
    const auto pos = buffer_.find('\n');
    if (pos == std::string::npos) return std::nullopt;

    std::string line = buffer_.substr(0, pos);
    buffer_.erase(0, pos + 1);

    // trim trailing '\r'
    if (!line.empty() && line.back() == '\r') line.pop_back();
    return line;
  }

  struct RMC {
    bool valid{false};         // 'A' valid, 'V' void
    double lat_deg{0.0};
    double lon_deg{0.0};
    double sog_mps{0.0};       // speed over ground m/s
    double cog_deg{0.0};       // course over ground deg
    bool has_pos{false};
    bool has_sog{false};
    bool has_cog{false};
  };

  struct GGA {
    bool has_pos{false};
    double lat_deg{0.0};
    double lon_deg{0.0};
    int fix_quality{0}; // 0 invalid
    int sats{0};
  };

  static std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) elems.push_back(item);
    return elems;
  }

  static double nmea_to_decimal(const std::string &coord, const std::string &hemisphere) {
    if (coord.empty()) return 0.0;
    const double val = std::stod(coord);          // ddmm.mmmm or dddmm.mmmm
    const int deg = static_cast<int>(val / 100);  // dd or ddd
    const double min = val - deg * 100;
    double dec = deg + (min / 60.0);
    if (hemisphere == "S" || hemisphere == "W") dec = -dec;
    return dec;
  }

  // Parse a single NMEA line. Returns RMC/GGA when recognized.
  std::optional<RMC> parse_rmc(const std::string& line) {
    if (line.rfind("$GPRMC", 0) != 0 && line.rfind("$GNRMC", 0) != 0) return std::nullopt;

    const auto f = split(line, ',');
    // $GPRMC,hhmmss.sss,A,llll.ll,a,yyyyy.yy,a,sog,cog,ddmmyy,...
    if (f.size() < 10) return std::nullopt;

    RMC out{};
    out.valid = (f[2] == "A");

    if (!f[3].empty() && !f[4].empty() && !f[5].empty() && !f[6].empty()) {
      out.lat_deg = nmea_to_decimal(f[3], f[4]);
      out.lon_deg = nmea_to_decimal(f[5], f[6]);
      out.has_pos = true;
    }

    // Speed over ground in knots (field 7)
    if (!f[7].empty()) {
      const double sog_knots = std::stod(f[7]);
      out.sog_mps = sog_knots * 0.514444; // knots -> m/s
      out.has_sog = true;
    }

    // Course over ground in degrees (field 8)
    if (!f[8].empty()) {
      out.cog_deg = std::stod(f[8]);
      out.has_cog = true;
    }

    return out;
  }

  std::optional<GGA> parse_gga(const std::string& line) {
    if (line.rfind("$GPGGA", 0) != 0 && line.rfind("$GNGGA", 0) != 0) return std::nullopt;

    const auto f = split(line, ',');
    // $GPGGA,hhmmss.sss,lat,NS,lon,EW,fix,sats,...
    if (f.size() < 8) return std::nullopt;

    GGA out{};
    if (!f[6].empty()) out.fix_quality = std::stoi(f[6]);
    if (!f[7].empty()) out.sats = std::stoi(f[7]);

    if (!f[2].empty() && !f[3].empty() && !f[4].empty() && !f[5].empty()) {
      out.lat_deg = nmea_to_decimal(f[2], f[3]);
      out.lon_deg = nmea_to_decimal(f[4], f[5]);
      out.has_pos = true;
    }
    return out;
  }

private:
  int fd_{-1};
  std::string port_;
  speed_t baud_;
  int timeout_ms_{500};

  std::string buffer_;
};

class PavbotGPSNode : public rclcpp::Node {
public:
  PavbotGPSNode() : Node("pavbot_gps")
  {
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    timeout_ms_ = this->declare_parameter<int>("timeout_ms", 500);
    publish_hz_ = this->declare_parameter<double>("publish_hz", 10.0);

    // QoS: GPS is a live sensor stream; transient_local is not appropriate here.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", qos);
    cog_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gps/cog_deg", qos);
    sog_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gps/sog_mps", qos);

    gps_.configure(port_, B4800, timeout_ms_);

    if (gps_.open_port()) {
      RCLCPP_INFO(this->get_logger(), "Opened GPS on %s @ 4800 baud", port_.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open GPS %s: %s", port_.c_str(), strerror(errno));
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PavbotGPSNode::tick, this));
  }

  ~PavbotGPSNode() override {
    gps_.close_port();
  }

private:
  void tick() {
    // Read bytes
    gps_.pump();

    // Consume all available complete lines each tick
    while (true) {
      auto line_opt = gps_.next_line();
      if (!line_opt) break;
      const std::string& line = *line_opt;
      if (line.empty() || line[0] != '$') continue;

      // Prefer RMC (has validity + sog/cog)
      if (auto rmc = gps_.parse_rmc(line)) {
        handle_rmc(*rmc);
        continue;
      }
      if (auto gga = gps_.parse_gga(line)) {
        handle_gga(*gga);
        continue;
      }
    }

    // Optionally: if we haven't seen a valid fix in a while, publish NO_FIX status.
    // (Not strictly required for your waypoint follower, but helpful.)
    const double fix_age = (this->now() - last_fix_time_).seconds();
    if (fix_age > 2.0) {
      // Could publish a NO_FIX status fix if desired; leaving silent is fine.
    }
  }

  void handle_rmc(const GPS::RMC& rmc) {
    if (!rmc.valid) {
      // Invalid / void: you can publish NO_FIX if desired
      return;
    }
    if (!rmc.has_pos) return;

    publish_fix(rmc.lat_deg, rmc.lon_deg, /*has_fix=*/true);

    if (rmc.has_cog) {
      std_msgs::msg::Float32 cog;
      cog.data = static_cast<float>(rmc.cog_deg);
      cog_pub_->publish(cog);
    }
    if (rmc.has_sog) {
      std_msgs::msg::Float32 sog;
      sog.data = static_cast<float>(rmc.sog_mps);
      sog_pub_->publish(sog);
    }
  }

  void handle_gga(const GPS::GGA& gga) {
    // Only use GGA if it indicates a valid fix quality and has position.
    if (!gga.has_pos) return;
    const bool has_fix = (gga.fix_quality > 0);
    if (!has_fix) return;

    // If RMC is present too, you'll publish fix from RMC as well; that's fine.
    publish_fix(gga.lat_deg, gga.lon_deg, /*has_fix=*/true);
  }

  void publish_fix(double lat_deg, double lon_deg, bool has_fix) {
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = this->now();
    fix.header.frame_id = "gps_link";

    if (has_fix) {
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    } else {
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    }

    fix.latitude = lat_deg;
    fix.longitude = lon_deg;
    fix.altitude = 0.0; // BU-353S4 can provide altitude in GGA if you want; optional.

    // Covariance: unknown -> set type UNKNOWN. (You can set rough values later.)
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    // Only publish when changed materially (reduces spam)
    const bool changed =
      (!has_last_fix_) ||
      (std::abs(fix.latitude - last_fix_lat_) > 1e-9) ||
      (std::abs(fix.longitude - last_fix_lon_) > 1e-9);

    if (changed) {
      fix_pub_->publish(fix);
      last_fix_time_ = fix.header.stamp;
      has_last_fix_ = true;
      last_fix_lat_ = fix.latitude;
      last_fix_lon_ = fix.longitude;

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "GPS fix: lat=%.8f lon=%.8f", fix.latitude, fix.longitude);
    }
  }

  // Params
  std::string port_;
  int timeout_ms_{500};
  double publish_hz_{10.0};

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cog_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sog_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // GPS
  GPS gps_;

  // State
  rclcpp::Time last_fix_time_{0,0,RCL_ROS_TIME};
  bool has_last_fix_{false};
  double last_fix_lat_{0.0};
  double last_fix_lon_{0.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PavbotGPSNode>());
  rclcpp::shutdown();
  return 0;
}
