# Minimum cmake version -> current version
cmake_minimum_required(VERSION 3.5)

# Project name
project(GNSS VERSION 1.0 LANGUAGES CXX C)

# Create executable from source file (so compiler can find .cpp files)
add_executable(${PROJECT_NAME} 
    ${PROJECT_SOURCE_DIR}/main.c 
    ${PROJECT_SOURCE_DIR}/src/GNSSDriver.c
)

# Add include directory (so compiler can find .h files)
target_include_directories(${PROJECT_NAME} PRIVATE inc)
