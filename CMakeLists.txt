cmake_minimum_required(VERSION 3.10)
project(pid-core C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra)

add_library(pid STATIC src/pid.c)
target_include_directories(pid PUBLIC include)
