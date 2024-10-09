#ifndef Jbd_BMS_STATUS_H_
#define Jbd_BMS_STATUS_H_

#include <rclcpp/rclcpp.hpp>
//#include <ros/ros.h>
#include <serial/serial.h>
//#include <sensor_msgs/Imu.h>
#include <sensor_msgs/msg/imu.h>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <vector>
#include <string.h>
#include <unistd.h>
#include <chrono>
#include <boost/format.hpp>

//#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include "jbd_bms_msg/msg/jbd_status.hpp"

using namespace std::chrono;

namespace iqr {
  class JbdBmsStatus : public rclcpp::Node {
  public:
    uint8_t cmd_status_ = 0x03, cmd_voltage_ = 0x04,
      cmd_status_sum_ = 0xFD, cmd_voltage_sum_ = 0xFC;
    std::vector<uint8_t> buffer_all_, buffer_vol_;
    int looprate_;

    //JbdBmsStatus(ros::NodeHandle& nod);
    JbdBmsStatus(std::string nodename);
    bool initPort();
    std::vector<uint8_t> dataRead(uint8_t date_type, uint8_t check_sum_write); // , std::vector<uint8_t> buffer);
    void dataParsing(std::vector<uint8_t>& all, std::vector<uint8_t>& vol);

  private:
    jbd_bms_msg::msg::JbdStatus jbd_status_;
    serial::Serial bms_ser_;
    rclcpp::Time time_now_;
    //ros::Time time_now_;
    bool findpack = false; 
    std::vector<int> error_id_;
    rclcpp::Publisher<jbd_bms_msg::msg::JbdStatus>::SharedPtr jbd_pub_;



  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr residual_capacity_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr design_capacity_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr cycle_index_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr date_production_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr status_balance_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr status_protect_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr version_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr rsoc_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr status_mos_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr cell_number_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr ntc_number_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cell_voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ntc_temperature_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr error_id_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_info_pub_;

    //ros::Publisher jbd_pub_;
    std::string port_, frame_id_, date_production_string_, /* path_name_, */ node_name_; 
    float ntf_data_[10], cell_[30];
    float voltage_, current_;
    uint8_t buffer_write_[7];
    uint16_t date_production_int_, status_protect_, version_, status_mos_;
    uint32_t status_balance_;
    int baudrate_, cell_number_, residual_capacity_, design_capacity_, cycle_index_, rsoc_,
      ntc_, ntc_number_, day_production_, month_production_, year_production_; /*position_; */

    // diagnostic  update
    diagnostic_updater::Updater diagnostic_updater_;

    void jbdCallback();

    void publishStatusMessages(jbd_bms_msg::msg::JbdStatus status);

    void BMSDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status);
  };
} // namespace iqr
#endif // HONFU_BMS_STATUS_H_
