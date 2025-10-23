# The following lines of boilerplate have to be in your project's
# CMakeLists.txt in order to use ESP-IDF build system
cmake_minimum_required(VERSION 3.16)

# Set the project name
set(PROJECT_NAME "self-balancing-robot")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(${PROJECT_NAME})

idf_component_register(SRCS "main.cpp" "motor_driver.c" "mpu6050.c" "encoder.c" 
                            "kalman_filter.c" "pid_controller.c" "balance_control.c"
                    INCLUDE_DIRS "../include")