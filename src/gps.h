// In future I add re-import blocking and cleaner interfacing;
//  good for now :)

#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <cerrno>
#include <utility>

class GPS {
  public:
    GPS(std::string port = "/dev/ttyUSB0", speed_t baud = B4800, int timeout_ms = 500)
      : port(port), baud(baud), timeout_ms(timeout_ms) {
        fd = -1;
    }

    void configure(std::string port, speed_t baud, int timeout_ms) {
        this->port = port;
        this->baud = baud;
        this->timeout_ms = timeout_ms;
    }

    bool open() {
        fd = ::open(port.c_str(), O_RDONLY | O_NOCTTY);
        if (fd < 0) {
            return false;
        } 

        struct termios tty{};
        tcgetattr(fd, &tty);

        cfsetospeed(&tty, baud);
        cfsetispeed(&tty, baud);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = timeout_ms / 100;

        tcsetattr(fd, TCSANOW, &tty);
        return true;
    }

    std::string readSerial() {
        char buf[256];
    
        int n = read(this->fd, buf, sizeof(buf) - 1);
        if (n <= 0) {
            return {};
        }

        buf[n] = '\0';
        return std::string(buf);
    }

     bool readLatLng(std::pair<float, float> &latLng) {
        std::string sentence = readSerial();
        if (sentence.size() < 6) return false; // Dud

        // Packet may be one of several structures of GPS data
        if (sentence.substr(0,6) == "$GPRMC") {
        auto fields = split(sentence, ',');
        if (fields.size() < 7) return false; // Incomplete data; untrusted
        if (fields[2] != "A") return false; // Not "A"ctive ("V"oid)
        latLng.first = nmeaToDecimal(fields[3], fields[4]);
        latLng.second = nmeaToDecimal(fields[5], fields[6]);
        return true;
        }

        else if (sentence.substr(0,6) == "$GPGGA") {
        auto fields = split(sentence, ',');
        if (fields.size() < 6) return false; // Incomplete data; untrusted
        int fix = std::stoi(fields[6]);
        if (fix == 0) return false; // 0 ->  NO FIX
        latLng.first = nmeaToDecimal(fields[2], fields[3]);
        latLng.second = nmeaToDecimal(fields[4], fields[5]);
        return true;
        }

        return false;
    }

    bool close() {
        return false;
    }

  private:
    int fd;
    std::string port;
    speed_t baud;
    int timeout_ms;

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
};