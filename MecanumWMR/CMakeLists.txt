cmake_minimum_required(VERSION 3.18)
project(MecanumWMR)

add_library(MecanumWMR MecanumWMR.cpp i2c_master.cpp MPU9250.cpp TrajectoryGenerator.cpp)
target_link_libraries(MecanumWMR PUBLIC pigpiod_if2)
target_compile_options(MecanumWMR PRIVATE -Wall -g -pthread -lpigpiod_if2 -lrt -lm)

add_executable(main main.cpp)
target_link_libraries(main MecanumWMR)