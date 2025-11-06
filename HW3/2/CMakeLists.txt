cmake_minimum_required(VERSION 3.15)
project(robot LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find GLFW
find_package(glfw3 REQUIRED)

set(MUJOCO_FRAMEWORKS "/Applications/MuJoCo.app/Contents/Frameworks" CACHE PATH "MuJoCo frameworks dir")
set(MUJOCO_VERSION_DIR "${MUJOCO_FRAMEWORKS}/mujoco.framework/Versions/A")
set(MUJOCO_DYLIB "${MUJOCO_VERSION_DIR}/libmujoco.3.3.6.dylib" CACHE FILEPATH "Path to MuJoCo dylib")

# Add executables
add_executable(robot robot.cpp)
add_executable(robot_pd_control robot_pd_control.cpp)

# Common compile options
target_compile_options(robot PRIVATE
        "-F${MUJOCO_FRAMEWORKS}"
        "-iframework${MUJOCO_FRAMEWORKS}"
)
target_compile_options(robot_pd_control PRIVATE
        "-F${MUJOCO_FRAMEWORKS}"
        "-iframework${MUJOCO_FRAMEWORKS}"
)

# Include directories
target_include_directories(robot PRIVATE "${MUJOCO_VERSION_DIR}/Headers")
target_include_directories(robot_pd_control PRIVATE
    "${MUJOCO_VERSION_DIR}/Headers"
)

# Link libraries
target_link_libraries(robot PRIVATE "${MUJOCO_DYLIB}")
target_link_libraries(robot_pd_control PRIVATE 
    "${MUJOCO_DYLIB}"
    glfw
)

get_filename_component(_mjlib_dir "${MUJOCO_DYLIB}" DIRECTORY)

set_target_properties(robot PROPERTIES
        BUILD_RPATH   "${_mjlib_dir}"
        INSTALL_RPATH "${_mjlib_dir}"
)

set_target_properties(robot_pd_control PROPERTIES
        BUILD_RPATH   "${_mjlib_dir}"
        INSTALL_RPATH "${_mjlib_dir}"
)