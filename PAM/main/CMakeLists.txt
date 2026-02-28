idf_component_register(
    SRCS 
        "main.c"
        
        "drivers/pump/hal_pump.c"
        "drivers/valves/hal_valves.c"
        "drivers/as5600/as5600.c"
        "drivers/pressure/press.c"
        
        "algorithm/kalman/kalman.c"
        "algorithm/pid/pid.c"
        
        "app/arm_control/arm_control.c"
    
    INCLUDE_DIRS 
        "."  
       
        "drivers/pump"
        "drivers/valves"
        "drivers/as5600"
        "drivers/pressure"
    
        "algorithm/kalman"
        "algorithm/pid"
        
        "app/arm_control"
)