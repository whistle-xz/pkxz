idf_component_register(
    SRCS 
        "main.c"
        
        "drivers/hal_pump/hal_pump.c"
        "drivers/hal_valves/hal_valves.c"
        "drivers/as5600/as5600.c"
        "drivers/pressure/press.c"
        
        "algorithm/kalman/kalman.c"
        "algorithm/pid/pid.c"
        
        "app/arm_control/arm_control.c"
    
    INCLUDE_DIRS 
        "."             
        "drivers/hal_pump"
        "drivers/hal_valves"
        "drivers/as5600"
        "drivers/pressure"
        "algorithm/kalman"
        "algorithm/pid"
        "app/arm_control"
)