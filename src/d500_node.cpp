#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

class D500Node : public rclcpp::Node
{
public:
  D500Node() : Node("d500_node")
  {
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baudrate_ = this->declare_parameter<int>("baudrate", 230400);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "lidar_link");
    topic_name_ = this->declare_parameter<std::string>("topic_name", "/scan");

    range_min_ = this->declare_parameter<double>("range_min", 0.03);
    range_max_ = this->declare_parameter<double>("range_max", 12.0);
    scan_frequency_ = this->declare_parameter<double>("scan_frequency", 10.0);

    clockwise_ = this->declare_parameter<bool>("clockwise", true);
    angle_offset_deg_ = this->declare_parameter<double>("angle_offset_deg", 0.0);
    bins_ = this->declare_parameter<int>("bins", 720);

    if (bins_ < 90) {
      bins_ = 360;
    }

    ranges_.assign(bins_, std::numeric_limits<float>::infinity());
    intensities_.assign(bins_, 0.0f);

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, 10);

    if (!openSerial()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_.c_str());
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "LDLiDAR D500 decoder started on %s at %d baud",
      port_.c_str(),
      baudrate_
    );

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2),
      std::bind(&D500Node::readLoop, this)
    );
  }

  ~D500Node()
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
  static constexpr uint8_t HEADER = 0x54;
  static constexpr uint8_t VER_LEN = 0x2C;
  static constexpr int POINTS_PER_PACKET = 12;
  static constexpr int PACKET_SIZE = 47;

  static uint16_t readU16LE(const std::vector<uint8_t>& data, size_t index)
  {
    return static_cast<uint16_t>(data[index]) |
           static_cast<uint16_t>(data[index + 1] << 8);
  }

  static uint8_t crc8(const uint8_t* data, size_t length)
  {
    uint8_t crc = 0x00;

    for (size_t i = 0; i < length; ++i) {
      crc ^= data[i];

      for (int bit = 0; bit < 8; ++bit) {
        if (crc & 0x80) {
          crc = static_cast<uint8_t>((crc << 1) ^ 0x4D);
        } else {
          crc = static_cast<uint8_t>(crc << 1);
        }
      }
    }

    return crc;
  }

  bool openSerial()
  {
    serial_fd_ = open(port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);

    if (serial_fd_ < 0) {
      return false;
    }

    termios tty{};

    if (tcgetattr(serial_fd_, &tty) != 0) {
      return false;
    }

    cfmakeraw(&tty);

    speed_t speed = B230400;

    if (baudrate_ != 230400) {
      RCLCPP_WARN(
        this->get_logger(),
        "Only 230400 is explicitly configured in this node. Using B230400 anyway."
      );
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      return false;
    }

    tcflush(serial_fd_, TCIFLUSH);

    return true;
  }

  void readLoop()
  {
    if (serial_fd_ < 0) {
      return;
    }

    uint8_t buffer[512];
    const int n = read(serial_fd_, buffer, sizeof(buffer));

    if (n <= 0) {
      return;
    }

    rx_buffer_.insert(rx_buffer_.end(), buffer, buffer + n);
    parsePackets();
  }

  void parsePackets()
  {
    while (rx_buffer_.size() >= PACKET_SIZE) {
      size_t start = 0;

      while (
        start + 1 < rx_buffer_.size() &&
        !(rx_buffer_[start] == HEADER && rx_buffer_[start + 1] == VER_LEN)
      ) {
        ++start;
      }

      if (start > 0) {
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + start);
      }

      if (rx_buffer_.size() < PACKET_SIZE) {
        return;
      }

      if (rx_buffer_[0] != HEADER || rx_buffer_[1] != VER_LEN) {
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }

      const uint8_t calculated_crc = crc8(rx_buffer_.data(), PACKET_SIZE - 1);
      const uint8_t received_crc = rx_buffer_[PACKET_SIZE - 1];

      if (calculated_crc != received_crc) {
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }

      parseFrame(rx_buffer_);
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + PACKET_SIZE);
    }
  }

  void parseFrame(const std::vector<uint8_t>& packet)
  {
    const uint16_t speed = readU16LE(packet, 2);
    const uint16_t start_angle_raw = readU16LE(packet, 4);
    const uint16_t end_angle_raw = readU16LE(packet, 42);

    double start_angle_deg = static_cast<double>(start_angle_raw) / 100.0;
    double end_angle_deg = static_cast<double>(end_angle_raw) / 100.0;

    double diff = end_angle_deg - start_angle_deg;

    if (diff < 0.0) {
      diff += 360.0;
    }

    const double step = diff / static_cast<double>(POINTS_PER_PACKET - 1);

    bool crossed_zero = false;

    if (has_last_angle_) {
      if (last_packet_end_angle_deg_ > 300.0 && end_angle_deg < 60.0) {
        crossed_zero = true;
      }
    }

    if (crossed_zero) {
      publishScan(speed);
      resetScan();
    }

    for (int i = 0; i < POINTS_PER_PACKET; ++i) {
      const size_t offset = 6 + i * 3;

      const uint16_t distance_mm = readU16LE(packet, offset);
      const uint8_t intensity = packet[offset + 2];

      double angle_deg = start_angle_deg + step * static_cast<double>(i);

      if (angle_deg >= 360.0) {
        angle_deg -= 360.0;
      }

      addPoint(angle_deg, distance_mm, intensity);
    }

    last_packet_end_angle_deg_ = end_angle_deg;
    has_last_angle_ = true;
  }

  void addPoint(double angle_deg, uint16_t distance_mm, uint8_t intensity)
  {
    if (distance_mm == 0) {
      return;
    }

    double range_m = static_cast<double>(distance_mm) / 1000.0;

    if (range_m < range_min_ || range_m > range_max_) {
      return;
    }

    if (clockwise_) {
      angle_deg = 360.0 - angle_deg;
    }

    angle_deg += angle_offset_deg_;

    while (angle_deg >= 360.0) {
      angle_deg -= 360.0;
    }

    while (angle_deg < 0.0) {
      angle_deg += 360.0;
    }

    double angle_rad = angle_deg * M_PI / 180.0;

    if (angle_rad > M_PI) {
      angle_rad -= 2.0 * M_PI;
    }

    const double angle_min = -M_PI;
    const double angle_increment = (2.0 * M_PI) / static_cast<double>(bins_);

    int index = static_cast<int>((angle_rad - angle_min) / angle_increment);

    if (index < 0 || index >= bins_) {
      return;
    }

    if (!std::isfinite(ranges_[index]) || range_m < ranges_[index]) {
      ranges_[index] = static_cast<float>(range_m);
      intensities_[index] = static_cast<float>(intensity);
    }
  }

  void publishScan(uint16_t speed)
  {
    sensor_msgs::msg::LaserScan scan;

    scan.header.stamp = this->now();
    scan.header.frame_id = frame_id_;

    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = static_cast<float>((2.0 * M_PI) / static_cast<double>(bins_));

    scan.time_increment = 0.0f;
    scan.scan_time = static_cast<float>(1.0 / scan_frequency_);

    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);

    scan.ranges = ranges_;
    scan.intensities = intensities_;

    scan_pub_->publish(scan);

    static int counter = 0;
    counter++;

    if (counter % 50 == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Published /scan | speed_raw=%u | bins=%d",
        speed,
        bins_
      );
    }
  }

  void resetScan()
  {
    std::fill(ranges_.begin(), ranges_.end(), std::numeric_limits<float>::infinity());
    std::fill(intensities_.begin(), intensities_.end(), 0.0f);
  }

private:
  std::string port_;
  std::string frame_id_;
  std::string topic_name_;

  int baudrate_;
  int serial_fd_{-1};
  int bins_;

  double range_min_;
  double range_max_;
  double scan_frequency_;
  double angle_offset_deg_;

  bool clockwise_;

  bool has_last_angle_{false};
  double last_packet_end_angle_deg_{0.0};

  std::vector<uint8_t> rx_buffer_;
  std::vector<float> ranges_;
  std::vector<float> intensities_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<D500Node>());
  rclcpp::shutdown();
  return 0;
}
