idf_component_register(SRCS "main.c" 
                            "motor_driver.c" 
                            "mpu6050.c" 
                            "encoder.c" 
                            "balance_control.c" 
                            "pid_controller.c"
                            "web_interface.c"
                    INCLUDE_DIRS "../include")