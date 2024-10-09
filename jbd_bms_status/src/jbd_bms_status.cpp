#include "jbd_bms_status.h"

// Error messages output to /diagnostic
const std::string error_info[14] = {
  // Messages for protection status 
  "Cell Overvoltage Protection", "Cell Undervoltage Protection",
  "Pack Overvoltage Protection", "Pack Undervoltage Protection", 
  "Overtemperature Protection while Charging", "Cryogenic Protection while Charging", 
  "Overtemperature Protection while Discharging", "Cryogenic Protection while Discharging", 
  "Overcurrent Protection while Charging", "Overcurrent Protection while Discharging", 
  "Short Circuit Protection", "Front-end IC Error", "MosFET Software Lock-in",
  // Generic communication error
  "Unable to Open Port"
};  

//iqr::JbdBmsStatus::JbdBmsStatus(ros::NodeHandle& nod) {
iqr::JbdBmsStatus::JbdBmsStatus(std::string nodename) :
  Node("jbd_bms_status"),
  diagnostic_updater_(this)
{
  buffer_write_[0] = 0xDD;    // start byte
  buffer_write_[1] = 0xA5;    // mode (read data from BMS)
  buffer_write_[3] = 0x00;    // data length
  buffer_write_[4] = 0xFF;    // check sum   ((sum of data bytes) - 1) & 0xff
  buffer_write_[6] = 0x77;    // stop byte

#if 1
  // ROS2 node parameter definition
  port_ = declare_parameter<std::string>("port", "jbd_bms");
  frame_id_ = declare_parameter<std::string>("frame_id", "jbd_bms");
  looprate_ = declare_parameter<int>("looprate", 2);
  baudrate_ = declare_parameter<int>("baudrate", 9600);
  node_name_ = get_name();
  jbd_pub_ = create_publisher< jbd_bms_msg::msg::JbdStatus >("jbd_bms", 1);

  voltage_pub_ = this->create_publisher<std_msgs::msg::Float32>("voltage", 10);
  current_pub_ = this->create_publisher<std_msgs::msg::Float32>("current", 10);
  residual_capacity_pub_ = this->create_publisher<std_msgs::msg::Int32>("residual_capacity", 10);
  design_capacity_pub_ = this->create_publisher<std_msgs::msg::Int32>("design_capacity", 10);
  cycle_index_pub_ = this->create_publisher<std_msgs::msg::Int16>("cycle_index", 10);
  date_production_pub_ = this->create_publisher<std_msgs::msg::String>("date_production", 10);
  status_balance_pub_ = this->create_publisher<std_msgs::msg::UInt32>("status_balance", 10);
  status_protect_pub_ = this->create_publisher<std_msgs::msg::UInt16>("status_protect", 10);
  version_pub_ = this->create_publisher<std_msgs::msg::UInt16>("version", 10);
  rsoc_pub_ = this->create_publisher<std_msgs::msg::Int16>("rsoc", 10);
  status_mos_pub_ = this->create_publisher<std_msgs::msg::Int16>("status_mos", 10);
  cell_number_pub_ = this->create_publisher<std_msgs::msg::Int16>("cell_number", 10);
  ntc_number_pub_ = this->create_publisher<std_msgs::msg::Int16>("ntc_number", 10);
  cell_voltage_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("cell_voltage", 10);
  ntc_temperature_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("ntc_temperature", 10);
  error_id_pub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("error_id", 10);
  error_info_pub_ = this->create_publisher<std_msgs::msg::String>("error_info", 10);
#else
  // ROS1
  nod.param<std::string>("port", port_, "jbd_bms");
  nod.param<std::string>("frame_id", frame_id_, "jbd_bms");
  nod.param<int>("looprate", looprate_, 2);
  nod.param<int>("baudrate", baudrate_, 9600);
  path_name_ = std::string(ros::this_node::getName()); 
  position_ = path_name_.rfind('/');
  node_name_ = path_name_.substr(position_+1); 
  jbd_pub_ = nod.advertise<jbd_bms_msg::msg::JbdStatus>("jbd_bms", 1); 
#endif

  diagnostic_updater_.setHardwareID("jbd_bms");
  diagnostic_updater_.add("BMS", this, &iqr::JbdBmsStatus::BMSDiagnostic);
}

void iqr::JbdBmsStatus::jbdCallback() {
}

void iqr::JbdBmsStatus::BMSDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status) {
  boost::format key_format;

  status.add("Voltage (V)", jbd_status_.voltage);
  status.add("Current (A)", jbd_status_.current);
  status.add("ResidualCapacity (mAh)", jbd_status_.residual_capacity);
  status.add("DesignCapacity (mAh)", jbd_status_.design_capacity);
  status.add("CycleIndex", jbd_status_.cycle_index);
  status.add("DateProduction", jbd_status_.date_production);
  status.add("Version", jbd_status_.version);
  status.add("Rsoc (%)", jbd_status_.rsoc);

  status.add("CellNumber", jbd_status_.cell_number);
  for(int i=0; i<jbd_status_.cell_number; i++) {
    key_format = boost::format("Cell %1% voltage (V)") % i;
    status.add(key_format.str(), jbd_status_.cell_voltage[i]);
  }

  status.add("NtcNumber", jbd_status_.ntc_number);
  for(int i=0; i<jbd_status_.ntc_number; i++) {
    key_format = boost::format("Ntc %1% (℃)") % i;
    status.add(key_format.str(), jbd_status_.ntc_tem[i]);
  }

  // ROS_INFO("motor %s", ToBinary(motor_status_[kLeftMotor].status, 1).c_str());

  status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");

  for(int i=0; i<jbd_status_.error_info.size(); i++) {
    status.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, jbd_status_.error_info[i]);
  }
  // if((motor_status_[kLeftMotor].status)&0x01 == 0x01) {
  //   status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Amps Limit currently active");
  // }
  // if((motor_status_[kLeftMotor].status>>1)&0x01 == 0x01) {
  //   status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motor stalled");
  // }
  // if((motor_status_[kLeftMotor].status>>2)&0x01 == 0x01) {
  //   status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Loop Error detected");
  // }
  // //if(motor_status_[kLeftMotor].status&0x08 == 0x08) {
  // if((motor_status_[kLeftMotor].status>>3)&0x01 == 0x01) {
  //   status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Safety Stop active");
  // }
  // if((motor_status_[kLeftMotor].status>>4)&0x01 == 0x01) {
  //   status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Forward Limit triggered");
  // }
  // if((motor_status_[kLeftMotor].status>>5)&0x01 == 0x01) {
  //   status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Reverse Limit triggered");
  // }
  // if((motor_status_[kLeftMotor].status>>6)&0x01 == 0x01) {
  //   status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Amps Trigger activated");
  // }

}

bool iqr::JbdBmsStatus::initPort() {
  //ros::Rate loop_openport(0.2);
  rclcpp::Rate loop_openport(0.2);
  while(!bms_ser_.isOpen()) {
    RCLCPP_INFO(get_logger(), "%s:%d [%s] port_:%s baudrate_:%d",
		__FILE__, __LINE__, node_name_.c_str(), port_.c_str(), baudrate_);
    try{
      RCLCPP_INFO(get_logger(), "%s:%d [%s]port_:%s baudrate_:%d",
		  __FILE__, __LINE__, node_name_.c_str(), port_.c_str(), baudrate_);
      bms_ser_.setPort(port_);
      bms_ser_.setBaudrate(baudrate_);
      serial::Timeout t_out = serial::Timeout::simpleTimeout(1000);
      bms_ser_.setTimeout(t_out);
      bms_ser_.open();
      //ROS_INFO("[%s]Serial port initialized", node_name_.c_str());
      RCLCPP_INFO(get_logger(), "[%s]Serial port initialized", node_name_.c_str());
    }
    catch(serial::IOException& e) {
      RCLCPP_INFO(get_logger(), "%s:%d [%s]port_:%s baudrate_:%d",
		  __FILE__, __LINE__, node_name_.c_str(), port_.c_str(), baudrate_);
      //ROS_ERROR("[%s]Unable to open port, wait 5 secs and try again", node_name_.c_str());
      RCLCPP_ERROR(get_logger(), "[%s]Unable to open port, wait 5 secs and try again", node_name_.c_str());
      dataParsing(buffer_all_, buffer_vol_);
      loop_openport.sleep();
    }
  }
  return true;
}

void iqr::JbdBmsStatus::dataParsing(std::vector<uint8_t>& buffer_read,std::vector<uint8_t>& buffer_vol) {

  bool flag_buffer_read_complete = false;
  bool flag_buffer_vol_complete = false;

  //time_now_ = ros::Time::now();
  time_now_ = get_clock()->now();

  if(27 <= buffer_read.size()) {
    
    voltage_ = (buffer_read[4]<<8|buffer_read[5])/100.0;
    if(((buffer_read[6] & 0b10000000) >> 7) == 1) {
      current_ = ((buffer_read[6]<<8|buffer_read[7])-65535.0)/100.0;
    } else {
      current_ = (buffer_read[6]<<8|buffer_read[7])/100.0;
    }

    residual_capacity_ = (buffer_read[8]<<8|buffer_read[9])*10;
    design_capacity_ = (buffer_read[10]<<8|buffer_read[11])*10;
    cycle_index_ = (buffer_read[12]<<8|buffer_read[13]);
    date_production_int_ = (buffer_read[14]<<8|buffer_read[15]);
    status_balance_ = (buffer_read[16]<<8|buffer_read[17]|buffer_read[18]<<8|buffer_read[19]);
    status_protect_ = (buffer_read[20]<<8|buffer_read[21]);
    version_ = buffer_read[22];
    rsoc_ = buffer_read[23];
    status_mos_ = buffer_read[24];
    cell_number_ = buffer_read[25];
    ntc_number_ = buffer_read[26];
    
    if (27+ntc_number_*2 <= buffer_read.size()) {
      for(int i = 0; i < ntc_number_*2; i+=2) {
        ntf_data_[i/2] = ((buffer_read[27+i]<<8|buffer_read[28+i])-2731)/10.0;
      }
      flag_buffer_read_complete = true;
    } else {
      RCLCPP_WARN(get_logger(), "Incomplete NTC thermistor data in buffer_read.");  
    }

    day_production_ = (date_production_int_&0x1f);
    month_production_ = ((date_production_int_>>5)&0x0f);
    year_production_ = (2000+ (date_production_int_>>9));
    date_production_string_ = (boost::format("%04d-%02d-%02d") % year_production_ % month_production_% 
        day_production_).str();
   
    jbd_status_.header.stamp = time_now_;
    jbd_status_.header.frame_id = frame_id_;
    jbd_status_.voltage = voltage_;
    jbd_status_.current = current_;
    jbd_status_.residual_capacity = residual_capacity_;
    jbd_status_.design_capacity = design_capacity_;
    jbd_status_.cycle_index = cycle_index_;
    jbd_status_.date_production = date_production_string_;
    jbd_status_.status_balance = status_balance_;
    jbd_status_.status_protect = status_protect_;
    jbd_status_.version = version_;
    jbd_status_.rsoc = rsoc_;
    jbd_status_.status_mos = status_mos_;
    jbd_status_.cell_number = cell_number_;
    jbd_status_.ntc_number = ntc_number_;
    
    for(int i = 0; i < ntc_number_; ++i) {
      jbd_status_.ntc_tem.push_back(ntf_data_[i]);
    }
    
    for(int i = 0; i < 13; ++i) {   
      if((status_protect_ & (0x0001<<i)>>i)==1) {
        jbd_status_.error_id.push_back(i);
        jbd_status_.error_info.push_back(error_info[i]);  
      }
    }
  } else {
    RCLCPP_WARN(get_logger(), "buffer_read is incomplete.");
  }
  /////////////****************///////////////
  if(4 <= buffer_vol.size()) {   
    cell_number_ = buffer_vol[3]/2;
    if (5+cell_number_*2 <= buffer_vol.size()) {
      for(int i = 0; i < cell_number_*2; i+=2) {
        cell_[i/2] = (buffer_vol[4+i]<<8|buffer_vol[5+i])/1000.0;
        jbd_status_.cell_voltage.push_back(cell_[i/2]);       
      }
      flag_buffer_vol_complete = true;
    } else {
      RCLCPP_WARN(get_logger(), "Incomplete cell data in buffer_vol.");
    }
  } else {
    RCLCPP_WARN(get_logger(), "Size of buffer_vol insufficient.");
  }

  if(flag_buffer_read_complete && flag_buffer_vol_complete) {
    //RCLCPP_INFO(get_logger(), "Publishing JBD BMS status.");
    jbd_pub_->publish(jbd_status_);
  } else if(!bms_ser_.isOpen()) {
    RCLCPP_WARN(get_logger(), "Detected zero-size data.");
    jbd_status_.error_id.push_back(13);
    jbd_status_.error_info.push_back(error_info[13]);
    jbd_pub_->publish(jbd_status_);
    // jbd_status_.ErrorId.clear();
    // jbd_status_.ErrorInfo.clear();
  }

  publishStatusMessages(jbd_status_);

  // update diagnostic data
  //diagnostic_updater_.update();
  diagnostic_updater_.force_update();

  for(int i=0; i<jbd_status_.error_info.size(); i++) {
    RCLCPP_ERROR(get_logger(), "%s", jbd_status_.error_info[i].c_str());
    //ROS_ERROR("%s", jbd_status_.ErrorInfo[i].c_str());
  }

  buffer_vol_.clear();
  buffer_all_.clear();
  jbd_status_.ntc_tem.clear();
  jbd_status_.cell_voltage.clear();
  jbd_status_.error_id.clear();
  jbd_status_.error_info.clear();
}

void iqr::JbdBmsStatus::publishStatusMessages(jbd_bms_msg::msg::JbdStatus current_status) {
    
    // Publish voltage as Float32
    auto voltage_msg = std_msgs::msg::Float32();
    voltage_msg.data = current_status.voltage;
    voltage_pub_->publish(voltage_msg);

    // Publish current as Float32
    auto current_msg = std_msgs::msg::Float32();
    current_msg.data = current_status.current;
    current_pub_->publish(current_msg);

    // Publish residual_capacity as Int32
    auto residual_capacity_msg = std_msgs::msg::Int32();
    residual_capacity_msg.data = current_status.residual_capacity;
    residual_capacity_pub_->publish(residual_capacity_msg);

    // Publish design_capacity as Int32
    auto design_capacity_msg = std_msgs::msg::Int32();
    design_capacity_msg.data = current_status.design_capacity;
    design_capacity_pub_->publish(design_capacity_msg);

    // Publish cycle_index as Int16
    auto cycle_index_msg = std_msgs::msg::Int16();
    cycle_index_msg.data = current_status.cycle_index;
    cycle_index_pub_->publish(cycle_index_msg);

    // Publish date_production as String()
    auto date_production_msg = std_msgs::msg::String();
    date_production_msg.data = current_status.date_production;
    date_production_pub_->publish(date_production_msg);

    // Publish status_balance as UInt32
    auto status_balance_msg = std_msgs::msg::UInt32();
    status_balance_msg.data = current_status.status_balance;
    status_balance_pub_->publish(status_balance_msg);

    // Publish status_protect as UInt16
    auto status_protect_msg = std_msgs::msg::UInt16();
    status_protect_msg.data = current_status.status_protect;
    status_protect_pub_->publish(status_protect_msg);

    // Publish version as UInt16
    auto version_msg = std_msgs::msg::UInt16();
    version_msg.data = current_status.version;
    version_pub_->publish(version_msg);

    

    // Publish rsoc as Int16
    auto rsoc_msg = std_msgs::msg::Int16();
    rsoc_msg.data = current_status.rsoc;
    rsoc_pub_->publish(rsoc_msg);

    // Publish status_mos as Int16
    auto status_mos_msg = std_msgs::msg::Int16();
    status_mos_msg.data = current_status.status_mos;
    status_mos_pub_->publish(status_mos_msg);

    // Publish cell_number as Int16
    auto cell_number_msg = std_msgs::msg::Int16();
    cell_number_msg.data = current_status.cell_number;
    cell_number_pub_->publish(cell_number_msg);

    // Publish ntc_number as Int16
    auto ntc_number_msg = std_msgs::msg::Int16();
    ntc_number_msg.data = current_status.ntc_number;
    ntc_number_pub_->publish(ntc_number_msg);

    // Publish cell voltages as Float32MultiArray
    auto cell_voltages_msg = std_msgs::msg::Float32MultiArray();
    for (const auto& voltage : current_status.cell_voltage) {
        cell_voltages_msg.data.push_back(voltage);
    }
    cell_voltage_pub_->publish(cell_voltages_msg);

    // Publish NTC temperatures as Float32MultiArray
    auto ntc_tem_msg = std_msgs::msg::Float32MultiArray();
    for (const auto& temperature : current_status.ntc_tem) {
        ntc_tem_msg.data.push_back(temperature);
    }
    ntc_temperature_pub_->publish(ntc_tem_msg);

    // Publish status/error codes as Int64MultiArray
    auto error_ids_msg = std_msgs::msg::Int64MultiArray();
    auto error_info_msg = std_msgs::msg::String(); // For the concatenated error info

    for (size_t i = 0; i < current_status.error_id.size(); ++i) {
        error_ids_msg.data.push_back(current_status.error_id[i]);
        
        // Add corresponding error info if available
        if (i < current_status.error_info.size()) {
            // Concatenate error info, separated by a delimiter (e.g., comma)
            if (i > 0) {
                error_info_msg.data += ", "; // Add a comma if not the first error info
            }
            error_info_msg.data += current_status.error_info[i];
        }
    }

    // Publish the error IDs
    error_id_pub_->publish(error_ids_msg);

    // Publish the concatenated error info message
    if (!error_info_msg.data.empty()) {
        error_info_pub_->publish(error_info_msg);
    }

}

std::vector<uint8_t> iqr::JbdBmsStatus::dataRead(uint8_t date_type, uint8_t checksum_write) {
  std::vector<uint8_t> buffer;
  int index = 0;
  bool findpack;
  uint16_t buffer_sum = 0;
  uint16_t checksum_read = 0;

  buffer_write_[2] = date_type;
  buffer_write_[5] = checksum_write;
  findpack = false;    // 20240922HM added
  try{
    // clean remaining read data
    if(bms_ser_.available()) {   
      bms_ser_.read(buffer, bms_ser_.available());
    }
    // send status requenst to bms
    bms_ser_.write(buffer_write_,7); 
    //ros::Duration(0.1).sleep();
    rclcpp::sleep_for(300ms);
    // check status response from bms
    if(bms_ser_.available()) {   
      bms_ser_.read(buffer, bms_ser_.available());
      while(!findpack && index < buffer.size()) {
        if(buffer[index]==0xDD) {
          buffer.begin() = buffer.erase(buffer.begin(), buffer.begin()+index);
          checksum_read = buffer[buffer.size()-3]<<8|buffer[buffer.size()-2];
          for(int i = 0; i < buffer.size()-5; ++i) {
            buffer_sum += buffer[i+2];
          }
          buffer_sum = ~buffer_sum + 1;
          if(buffer_sum==checksum_read) {
            findpack = true;
          }
        }
        index += 1;
      }
    }
  }
  catch(serial::SerialException& e) {
    bms_ser_.close();
    initPort();
  }
  catch(serial::IOException& e) {
    bms_ser_.close();
    initPort();
  }
  if (!findpack) {
    RCLCPP_WARN(get_logger(), "Checksum error");
    buffer.clear();
    RCLCPP_WARN(get_logger(), "buffer cleared.");
  }        
  return buffer;
}

