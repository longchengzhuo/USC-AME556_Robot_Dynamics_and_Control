cmake_minimum_required(VERSION 3.15)
project(bar LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(MUJOCO_FRAMEWORKS "/Applications/MuJoCo.app/Contents/Frameworks" CACHE PATH "MuJoCo frameworks dir")
set(MUJOCO_VERSION_DIR "${MUJOCO_FRAMEWORKS}/mujoco.framework/Versions/A")
set(MUJOCO_DYLIB "${MUJOCO_VERSION_DIR}/libmujoco.3.3.6.dylib" CACHE FILEPATH "Path to MuJoCo dylib")


add_executable(bar bar.cpp)


target_compile_options(bar PRIVATE
        "-F${MUJOCO_FRAMEWORKS}"
        "-iframework${MUJOCO_FRAMEWORKS}"
)


target_include_directories(bar PRIVATE "${MUJOCO_VERSION_DIR}/Headers")


target_link_libraries(bar PRIVATE "${MUJOCO_DYLIB}")


get_filename_component(_mjlib_dir "${MUJOCO_DYLIB}" DIRECTORY)


set_target_properties(bar PROPERTIES
        BUILD_RPATH   "${_mjlib_dir}"
        INSTALL_RPATH "${_mjlib_dir}"
)