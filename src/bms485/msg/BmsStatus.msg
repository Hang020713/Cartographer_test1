std_msgs/Header header
float32[] cell_voltages       # Volts (0.001 V resolution)
float32[] cell_temperatures   # Kelvin (0.1 K resolution)
float32   charge_current      # Amps
float32   discharge_current   # Amps
float32   module_voltage      # Volts
uint16    soc                 # percent
uint32    total_capacity      # mAh