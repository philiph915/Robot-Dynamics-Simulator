cmake_minimum_required(VERSION 3.10)
project(Robot_Dynamics_Simulator)

# Use C++17
set(CMAKE_CXX_STANDARD 17)

# Add all source files
file(GLOB SRC_FILES src/*.cpp)

# Add executable
add_executable(Robot_Dynamics_Simulator ${SRC_FILES})

# Set the output file name (without changing target name)
set_target_properties(Robot_Dynamics_Simulator PROPERTIES OUTPUT_NAME "Robot Dynamics Simulator")

# Add your own project headers (warnings enabled)
target_include_directories(Robot_Dynamics_Simulator PRIVATE include)

# Add external headers (warnings suppressed)
target_include_directories(Robot_Dynamics_Simulator SYSTEM PRIVATE external)

