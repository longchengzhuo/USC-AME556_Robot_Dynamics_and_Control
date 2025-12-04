cmake_minimum_required(VERSION 3.15)
project(robot LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

# 1. Find Packages
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

# 2. MuJoCo Settings
set(MUJOCO_FRAMEWORKS "/Applications/MuJoCo.app/Contents/Frameworks" CACHE PATH "MuJoCo frameworks dir")
set(MUJOCO_VERSION_DIR "${MUJOCO_FRAMEWORKS}/mujoco.framework/Versions/A")
set(MUJOCO_DYLIB "${MUJOCO_VERSION_DIR}/libmujoco.3.3.6.dylib" CACHE FILEPATH "Path to MuJoCo dylib")

# 3. Helper Function
function(configure_target target_name)
    target_compile_options(${target_name} PRIVATE "-F${MUJOCO_FRAMEWORKS}" "-iframework${MUJOCO_FRAMEWORKS}")
    target_include_directories(${target_name} PRIVATE "${MUJOCO_VERSION_DIR}/Headers")
    # 链接 OsqpEigen, 它会自动链接 osqp
    target_link_libraries(${target_name} PRIVATE "${MUJOCO_DYLIB}" glfw Eigen3::Eigen OsqpEigen::OsqpEigen)

    get_filename_component(_mjlib_dir "${MUJOCO_DYLIB}" DIRECTORY)
    set_target_properties(${target_name} PROPERTIES BUILD_RPATH "${_mjlib_dir}" INSTALL_RPATH "${_mjlib_dir}")
endfunction()

# 4. Executables
# 将所有源文件加入编译列表
add_executable(robot_sim
        main.cpp
        SimulationUI.cpp
        BipedRobot.cpp
)

configure_target(robot_sim)