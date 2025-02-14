std_msgs/Header header

# voltage
float32 voltage

# current
float32 current

# residualCapacity
int32 residual_capacity

# DesignCapacity
int32 design_capacity

# CycleIndex
int16 cycle_index

# DateProduction
string date_production

# StatusBalance
uint32 status_balance

# StatusProtect
uint16 status_protect

# Version
uint16 version

# Rsoc
int16 rsoc

# StatusMos
int16 status_mos

# CellNumber
int16 cell_number

# NtcNumber
int16 ntc_number

# NtcTem
float64[] ntc_tem

# CellVoltage
float64[] cell_voltage

# ErrorId
int64[] error_id

# ErrorInfo
string[] error_info
