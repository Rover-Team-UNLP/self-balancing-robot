idf_component_register(SRCS "main.cpp" 
                            "motor_driver.c" 
                            "mpu6050.c" 
                            "encoder.c" 
                            "balance_control.c" 
                            "kalman_filter.c" 
                            "pid_controller.c"
                    INCLUDE_DIRS "../include")