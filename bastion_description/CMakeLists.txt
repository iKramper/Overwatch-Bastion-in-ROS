cmake_minimum_required(VERSION 3.8)
project(bastion_description)

# Set C++ standard:
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Compiler options:
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(DIRECTORY launch models rviz
    DESTINATION share/${PROJECT_NAME} 
)

find_package(ament_cmake REQUIRED)
ament_package() 