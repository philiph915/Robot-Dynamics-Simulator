cmake_minimum_required(VERSION 3.10)
project(Robot_Dynamics_Simulator)

# Use C++17
set(CMAKE_CXX_STANDARD 17)

# Define raylib paths
set(RAYLIB_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/external/raylib")
set(RAYLIB_DLL "${CMAKE_SOURCE_DIR}/libraylib.dll")
set(RAYLIB_LIBRARY "${CMAKE_SOURCE_DIR}/libraylib.dll.a")

# Add all source files
file(GLOB SRC_FILES src/*.cpp)

# Add executable
add_executable(Robot_Dynamics_Simulator ${SRC_FILES})

# Link libraries
target_link_libraries(Robot_Dynamics_Simulator ${RAYLIB_LIBRARY})

# Set the output file name (without changing target name)
set_target_properties(Robot_Dynamics_Simulator PROPERTIES OUTPUT_NAME "Robot Dynamics Simulator")

# Add headers
target_include_directories(Robot_Dynamics_Simulator PRIVATE include ${RAYLIB_INCLUDE_DIR}) # add raylib and project headers
target_include_directories(Robot_Dynamics_Simulator SYSTEM PRIVATE external)               # add eigen headers


# Ensure Windows finds the DLL at runtime (this automatically copies the raylib dll file to the build folder)
if (WIN32)
    add_custom_command(TARGET Robot_Dynamics_Simulator POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${RAYLIB_DLL}
        $<TARGET_FILE_DIR:Robot_Dynamics_Simulator>)
endif()