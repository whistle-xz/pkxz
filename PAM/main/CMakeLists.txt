idf_component_register(
    SRCS 
        "main.c"
        
        "drivers/hal_pump/hal_pump.c"
        "drivers/hal_valves/hal_valves.c"
        "drivers/pressure/press.c"
        "drivers/stm32_comm/stm32_comm.c"
        "drivers/as5600/as5600.c"
        
        "algorithm/kalman/kalman.c"
        "algorithm/pid/pid.c"
        
        "app/arm_control/arm_control.c"
        "app/cli/serial_debug.c"
    
    INCLUDE_DIRS 
        "."             
        "drivers/hal_pump"
        "drivers/hal_valves"
        "drivers/pressure"
        "drivers/stm32_comm"
        "drivers/as5600"
        
        "algorithm/kalman"
        "algorithm/pid"
        
        "app/arm_control"
        "app/cli"
)