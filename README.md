# jbd\_bms\_ros

### ROS node for Battery Management System by Jiabaida

## Overview

This ROS2 package reads Li-ion battery-related information returned by Jiabaida BMS and publishes it as topics.

The package publishes battery information such as SOC (State-Of-Charge) and error ID,
so that the main controller node can notice Low Voltage, High Temperature or any abnormality of Li-ion battery
and modify the control behaviour accordingly.

The package supports varying models of battery pack with differing number of cells or temperature sensors.


## Equipment Type

Jbd Power Lithium Battery\
![](https://github.com/I-Quotient-Robotics/iqr_jbd_bms/blob/master/type_pic/144283718.jpg)
![](https://github.com/I-Quotient-Robotics/iqr_jbd_bms/blob/master/type_pic/60348685.jpg)

## Environment

- Ubuntu 22.04
- ROS2 Humble

## Nodes

### `jbd_bms_status`

A ROS2 driver for battery management system from Jiabaida in China. It reads battery information and converts it to ROS2 topics.

#### Parameters

- `port` (type: `string`, default: `jbd_bms`)
    - Path of device file corresponding to the USB-UART adapter connecting to Jiabaida BMS

- `baudrate` (type: `int`, default: `9600`)
    - Baudrate for UART communication to Jiabaida BMS

- `loop_rate` (type: `int`, default: `2`)
    - Topic output rate [Hz]

- `frame_id` (type: `string`, default: `jbd_bms`)
    - Used in header of `jbd_bms`

## Topics

- `jbd_bms`
- `diagnostics`


###  `jbd_bms`

State of battery sent from Jiabaida BMS.

Type: `jbd_bms_msg/JbdStatus`

#### `jbd_bms_msg/JbdStatus`

```
std_msgs/Header header    # Header
float32 voltage           # Pack Voltage [V]
float32 current           # Pack Current [A] (positive:charge negative:discharge)
int32 residual_capacity   # Remaining Capacity [mAh]
int32 design_capacity     # Designed Capacity [mAh]
int16 cycle_index         # Number of Charge Cycles
string date_production    # Date of Production
uint32 status_balance     # bit field representing Balance Status of each cell    (1: balancing under way)
uint16 status_protect     # bit field representing Protection Status of each cell (1: protection trigger detected)
uint16 version            # Version (0x10 --> ver1.0)
int16 rsoc                # State-Of-Charge [%]
int16 status_mos          # MosFET status (0:OFF 1:discharge ON 2:charge ON 3:ON for both direction (discharge/charge)
int16 cell_number         # Number of cells connected to BMS
int16 ntc_number          # Number of thermistors connected
float64[] ntc_tem         # Temperature [C] (degree Celsius) of all thermistors connected
float64[] cell_voltage    # Cell voltage [V] of all cells connected
int64[] error_id          # Array of error id's collected from Protection Status
string[] error_info       # Array of error messages corresponding to error_id
```

##### Error ID

Note: `error_id` in `jbd_bms_msg/JbdStatus` is a variable array of error ID's. Length of `error_id` becomes 0 when no error is detected.

| error_id | description               |
|----------|------------------------------|
| 0        | Cell Overvoltage             |
| 1        | Cell Undervoltage            |
| 2        | Pack Overvoltage             |
| 3        | Pack Undervoltage            |
| 4        | Over-temp while charging     |
| 5        | Under-temp while charging    |
| 6        | Over-temp while discharging  |
| 7        | Under-temp while discharging |
| 8        | Overcurrent while charging   |
| 9        | Overcurrent while discharging|
| 10       | Short circuit                |
| 11       | Front-end IC error           |
| 12       | MosFET software lock-in      |
| 13       | Communication error          |



### `/diagnostics`

Diagnostics from Jiabaida BMS. Many human-readable messages on battery state.


## Build & Install

Execute following commandS to build and install the package.

<pre>
cd <i>your-workspace</i>/<i>your-source-directory</i>
git clone git@github.com:Futu-reADS/jbd_bms_ros.git
cd <i>your-workspace</i>
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
</pre>

## Udev rule

Udev rule file for USB-UART adaptor connecting to Jiabaida BMS:

```
KERNEL=="ttyUSB[0-9]",ATTRS{idVendor}=="10c4",ATTRS{idProduct}=="ea60",MODE:="0777",SYMLINK+="jbd_bms"
```

To install the udev rule:

```bash
sudo cp jbd_bms_driver/udev/10-jbd-bms.rules /etc/udev/rules.d/ or /lib/udec/rules.d
sudo udevadm control--reload-rules && udevadm trigger
```

## Run


<pre>
cd <i>your-workspace</i>
ros2 launch jbd_bms_status jbd_bms_status_launch.xml
</pre>


## Official website
http://www.jiabaida.com/

