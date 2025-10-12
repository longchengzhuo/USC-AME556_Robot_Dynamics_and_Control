cmake_minimum_required(VERSION 3.15)
project(log_cartpole LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- MuJoCo (DMG) paths ----
set(MUJOCO_FRAMEWORKS "/Applications/MuJoCo.app/Contents/Frameworks" CACHE PATH "MuJoCo frameworks dir")
set(MUJOCO_VERSION_DIR "${MUJOCO_FRAMEWORKS}/mujoco.framework/Versions/A")
set(MUJOCO_DYLIB "${MUJOCO_VERSION_DIR}/libmujoco.3.3.6.dylib" CACHE FILEPATH "Path to MuJoCo dylib")

if(NOT EXISTS "${MUJOCO_DYLIB}")
    message(FATAL_ERROR "MuJoCo dylib not found: ${MUJOCO_DYLIB}")
endif()
if(NOT EXISTS "${MUJOCO_VERSION_DIR}/Headers/mujoco.h")
    message(FATAL_ERROR "Umbrella header not found: ${MUJOCO_VERSION_DIR}/Headers/mujoco.h")
endif()

# ---- your program (把源文件名改成你的实际文件名) ----
add_executable(log_cartpole mujoco.cpp)

# 关键：编译期按“Framework 方式”找头（不用于链接）
target_compile_options(log_cartpole PRIVATE
        "-F${MUJOCO_FRAMEWORKS}"
        "-iframework${MUJOCO_FRAMEWORKS}"
)

# 备份：同时把 Headers 加到普通 include，便于 <mujoco.h> 被找到
target_include_directories(log_cartpole PRIVATE "${MUJOCO_VERSION_DIR}/Headers")

# 仅用 dylib 链接（不使用 -framework）
target_link_libraries(log_cartpole PRIVATE "${MUJOCO_DYLIB}")

# 运行时 rpath，确保能定位 dylib
get_filename_component(_mjlib_dir "${MUJOCO_DYLIB}" DIRECTORY)
set_target_properties(log_cartpole PROPERTIES
        BUILD_RPATH   "${_mjlib_dir}"
        INSTALL_RPATH "${_mjlib_dir}"
)
