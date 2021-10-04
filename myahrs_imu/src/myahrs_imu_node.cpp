#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <myahrs_imu/myahrs_plus.hpp>

#include "rclcpp/rclcpp.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace WithRobot;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class myahrsIMUNode : public rclcpp::Node, public iMyAhrsPlus
{
  public:
    myahrsIMUNode(std::string port="/dev/ttyACM0", int baud_rate=115200)
    : Node("myahrs_imu_node"),
      iMyAhrsPlus(port, baud_rate),
      count_(0)
    {
      port_ = port;
      baud_rate_ = baud_rate;
      imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/sample", 10);
      imu_mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
      timer_ = this-> create_wall_timer(10ms, std::bind(&myahrsIMUNode::timer_callback, this));
    }

    // ~MyAhrsDriverForROS()
    // {}

    bool initialize()
    {
      RCLCPP_INFO(this->get_logger(), "Establishing serial connection over port: %s with baud rate: %d\n", port_.c_str(), baud_rate_);
      bool start_check = start();
      //Euler angle(x, y, z axis)
      //IMU(linear_acceleration, angular_velocity, magnetic_field)
      bool cmd_binary_data_format_check = cmd_binary_data_format("EULER, IMU");
      // 100Hz
      bool cmd_divider_check = cmd_divider("1");
      // Binary and Continue mode
      bool cmd_mode_check = cmd_mode("BC");
      // RCLCPP_INFO(this->get_logger(), "start: %d, cmd_binary_data_format: %d, cmd_divider: %d, cmd_mode: %d\n", start_check, cmd_binary_data_format_check, cmd_divider_check, cmd_mode_check);

      if (!start_check || !cmd_binary_data_format_check || !cmd_divider_check || !cmd_mode_check)
      {
        return false;
      }
      else
      {
        return true;
      }
    }

    inline void get_data(SensorData& data)
    {
      LockGuard _l(lock_);
      data = sensor_data_;
    }

    inline SensorData get_data()
    {
      LockGuard _l(lock_);
      return sensor_data_;
    }

    void publish_topic(int sensor_id)
    {
      rclcpp::Time time = this->now();

      sensor_msgs::msg::Imu imu_msg;
      sensor_msgs::msg::MagneticField imu_mag_msg;

      double roll  =  sensor_data_.euler_angle.roll * convertor_d2r;
      double pitch = -sensor_data_.euler_angle.pitch * convertor_d2r;
      double yaw   = -sensor_data_.euler_angle.yaw * convertor_d2r;

      ImuData<float>& imu = sensor_data_.imu;

      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);

      imu_msg.header.stamp      =
      imu_mag_msg.header.stamp  = time;

      // orientation
      imu_msg.orientation.x = q[0];
      imu_msg.orientation.y = q[1];
      imu_msg.orientation.z = q[2];
      imu_msg.orientation.w = q[3];

      // original data used the g unit, convert to m/s^2
      imu_msg.linear_acceleration.x     =  imu.ax * convertor_g2a;
      imu_msg.linear_acceleration.y     = -imu.ay * convertor_g2a;
      imu_msg.linear_acceleration.z     = -imu.az * convertor_g2a;

      // original data used the degree/s unit, convert to radian/s
      imu_msg.angular_velocity.x     =  imu.gx * convertor_d2r;
      imu_msg.angular_velocity.y     = -imu.gy * convertor_d2r;
      imu_msg.angular_velocity.z     = -imu.gz * convertor_d2r;

      // original data used the uTesla unit, convert to Tesla
      imu_mag_msg.magnetic_field.x =  imu.mx / convertor_ut2t;
      imu_mag_msg.magnetic_field.y = -imu.my / convertor_ut2t;
      imu_mag_msg.magnetic_field.z = -imu.mz / convertor_ut2t;

      imu_pub_->publish(imu_msg);
      imu_mag_pub_->publish(imu_mag_msg);
    }

  private:
    void timer_callback()
    {
      // if (!initialized) {
      //   RCLCPP_INFO(this->get_logger(), "Initializing sensor...");
      //   initialized = this->initialize();
      //   if (initialized) {
      //     RCLCPP_INFO(this->get_logger(), "Done");
      //   }
      // }
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    }

    bool initialized = false;

    std::string port_;
    int baud_rate_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr imu_mag_pub_;
    size_t count_;

    Platform::Mutex lock_;
    SensorData sensor_data_;

    // std::string parent_frame_id_;
    // std::string frame_id_;
    // double linear_acceleration_stddev_;
    // double angular_velocity_stddev_;
    // double magnetic_field_stddev_;
    // double orientation_stddev_;

    static constexpr double convertor_g2a  = 9.80665;    // for linear_acceleration (g to m/s^2)
    static constexpr double convertor_d2r  = M_PI/180.0; // for angular_velocity (degree to radian)
    static constexpr double convertor_r2d  = 180.0/M_PI; // for easy understanding (radian to degree)
    static constexpr double convertor_ut2t = 1000000;    // for magnetic_field (uT to Tesla)
    static constexpr double convertor_c    = 1.0;        // for temperature (celsius)

    void OnSensorData(int sensor_id, SensorData data)
    {
      LockGuard _l(lock_);
      sensor_data_ = data;
      publish_topic(sensor_id);
    }

    void OnAttributeChange(int sensor_id, std::string attribute_name, std::string value)
    {
      printf("OnAttributeChange(id %d, %s, %s)\n", sensor_id, attribute_name.c_str(), value.c_str());
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // myahrsIMUNode node = std::make_shared<myahrsIMUNode>()
  std::string port = std::string("/dev/ttyACM0");
  int baud_rate    = 115200;
  std::shared_ptr<myahrsIMUNode> node_ptr = std::make_shared<myahrsIMUNode>(port, baud_rate);
  if (!node_ptr->initialize())
  {
    RCLCPP_ERROR(node_ptr->get_logger(), "initialize() returned false, please check your devices\n");
    return 0;
  }
  else
  {
    RCLCPP_INFO(node_ptr->get_logger(), "Initialization OK!\n");
  }

  rclcpp::spin(node_ptr);
  rclcpp::shutdown();
  return 0;
}
