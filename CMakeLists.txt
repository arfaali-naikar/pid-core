cmake_minimum_required(VERSION 3.10)
project(pid-core C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra)

add_library(pid STATIC src/pid.c)
target_include_directories(pid PUBLIC include)

enable_testing()

add_library(unity STATIC tests/unity/unity.c)
target_include_directories(unity PUBLIC tests/unity)

add_executable(test_pid tests/test_pid.c)
target_link_libraries(test_pid PRIVATE pid unity)

add_test(NAME test_pid COMMAND test_pid)

add_executable(motor_sim sim/main.c sim/motor.c)
target_link_libraries(motor_sim PRIVATE pid)
target_include_directories(motor_sim PRIVATE sim)
