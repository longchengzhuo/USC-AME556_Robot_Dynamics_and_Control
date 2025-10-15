# ==============================================================================
# 1. 项目基本设置 (Project Basic Setup)
# ==============================================================================

# 指定运行此 CMake 脚本所需的最低 CMake 版本。
# 如果用户的 CMake 版本低于 3.15，将会收到一个错误提示。
# 这确保了项目构建脚本的兼容性，因为不同版本的 CMake 可能有不同的命令或行为。
cmake_minimum_required(VERSION 3.15)

# 定义项目名称为 "log_cartpole"。
# 这个名称会用在很多地方，比如在 IDE (如 CLion, Visual Studio) 中显示的项目名。
# LANGUAGES CXX 表示这个项目主要使用 C++ 语言。
project(log_cartpole LANGUAGES CXX)

# 设置 C++ 标准为 C++17。
# CMAKE_CXX_STANDARD 指定了要使用的 C++ 标准版本号。
set(CMAKE_CXX_STANDARD 17)
# 强制要求必须使用指定的 C++ 标准。
# 如果编译器不支持 C++17，CMake 会报错，而不是降级到旧版本。
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# ==============================================================================
# 2. 依赖项路径设置 (Dependency Path Setup) - MuJoCo
# ==============================================================================

# ---- MuJoCo (DMG 安装方式) 的路径设置 ----
# 在 macOS 上，通过 .dmg 安装的 MuJoCo 应用，其核心文件（库、头文件）都存储在 .app 包内。
# 我们需要告诉 CMake 如何找到这些文件。

# 设置一个变量 MUJOCO_FRAMEWORKS，指向 MuJoCo.app 内部的 Frameworks 目录。
# 这是 macOS Framework 的标准路径结构。
# CACHE PATH 意味着这个变量会被存储在 CMake 的缓存中。用户可以在 CMake GUI 或 ccmake 中修改它，
# 这在需要将项目分享给他人，而他人的 MuJoCo 安装路径不同时非常有用。
# "MuJoCo frameworks dir" 是在 CMake GUI 中显示的描述性文字。
set(MUJOCO_FRAMEWORKS "/Applications/MuJoCo.app/Contents/Frameworks" CACHE PATH "MuJoCo frameworks dir")

# 基于 Frameworks 路径，构建指向特定版本 MuJoCo 库的路径。
# macOS 的 Framework 可以包含多个版本，'A' 通常是当前稳定版本的符号链接。
set(MUJOCO_VERSION_DIR "${MUJOCO_FRAMEWORKS}/mujoco.framework/Versions/A")

# 构建指向 MuJoCo 动态链接库 (.dylib) 文件的完整路径。
# 这是链接阶段需要用到的核心库文件。
# CACHE FILEPATH 同样是为了方便在缓存中修改。
set(MUJOCO_DYLIB "${MUJOCO_VERSION_DIR}/libmujoco.3.3.6.dylib" CACHE FILEPATH "Path to MuJoCo dylib")


# ==============================================================================
# 4. 定义可执行文件 (Define Executable)
# ==============================================================================

# ---- 定义你的程序 (把源文件名改成你的实际文件名) ----
# add_executable 命令创建一个可执行文件。
# 第一个参数 "log_cartpole" 是你想要生成的可执行文件的名字。
# 第二个参数 "mujoco.cpp" 是编译这个可执行文件所需的源文件。
# 如果有多个源文件，可以像这样列出: add_executable(my_app file1.cpp file2.cpp)
add_executable(log_cartpole mujoco.cpp)


# ==============================================================================
# 5. 编译与链接设置 (Compile and Link Settings)
# ==============================================================================

# ---- 编译设置 (告诉编译器如何找到头文件) ----

# 关键：在 macOS 上，使用 Framework 的标准方式是 -F 和 -iframework 编译选项。
# target_compile_options 命令给一个特定的目标（这里是 log_cartpole）添加编译选项。
# PRIVATE 关键字意味着这些选项只对这个目标本身生效，不会传递给链接到它的其他目标。
# "-F${MUJOCO_FRAMEWORKS}" 告诉编译器去哪里寻找 Framework 包。
# "-iframework${MUJOCO_FRAMEWORKS}" 告诉编译器要包含哪个 Framework。
target_compile_options(log_cartpole PRIVATE
        "-F${MUJOCO_FRAMEWORKS}"
        "-iframework${MUJOCO_FRAMEWORKS}"
)

# 备份：为了更好的 IDE 兼容性（例如 CLion 的代码提示），我们也把 Headers 目录显式地添加到 include 路径中。
# target_include_directories 命令将一个目录添加到目标的头文件搜索路径中。
# 这样，当代码中使用 #include <mujoco/mujoco.h> 时，编译器就能找到它。
target_include_directories(log_cartpole PRIVATE "${MUJOCO_VERSION_DIR}/Headers")


# ---- 链接设置 (告诉链接器要链接哪个库) ----

# 仅用 .dylib 的完整路径来链接，而不是使用 -framework mujoco 这种更“黑盒”的方式。
# 这样做更明确，能精确控制链接的库文件版本。
# target_link_libraries 命令将一个库链接到目标。
# 这里我们将之前定义的 MUJOCO_DYLIB 变量（指向 .dylib 文件的路径）链接到 log_cartpole。
target_link_libraries(log_cartpole PRIVATE "${MUJOCO_DYLIB}")


# ==============================================================================
# 6. 运行时路径设置 (Runtime Path (RPATH) Setup)
# ==============================================================================

# RPATH (Runtime Path) 是一个嵌入在可执行文件中的路径，用于告诉操作系统在运行时去哪里寻找动态链接库。
# 这在 macOS 和 Linux 上非常重要，可以避免 "library not found" 的运行时错误。

# 从 .dylib 的完整路径中提取出它所在的目录。
# _mjlib_dir 变量将被设置为 .dylib 文件所在的文件夹路径。
get_filename_component(_mjlib_dir "${MUJOCO_DYLIB}" DIRECTORY)

# 设置目标属性。这里我们为 log_cartpole 设置 RPATH 相关的属性。
# BUILD_RPATH 设置了在构建目录中运行可执行文件时的 RPATH。
# INSTALL_RPATH 设置了在安装后（如果定义了安装规则）运行可执行文件时的 RPATH。
# 我们都将它们设置为 MuJoCo 库所在的目录。
# 这样，无论你在哪里运行 log_cartpole，它都知道去这个路径下寻找 libmujoco.3.3.6.dylib。
set_target_properties(log_cartpole PROPERTIES
        BUILD_RPATH   "${_mjlib_dir}"
        INSTALL_RPATH "${_mjlib_dir}"
)