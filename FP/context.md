MuJoCo 双足机器人仿真项目上下文文档

1. 项目概述

本项目是一个基于 MuJoCo 物理引擎和 C++ 的双足机器人（Biped Robot）仿真框架。目标是实现机器人的运动控制（站立、行走等），并实施严格的物理约束监测。

当前进度：

已完成基础 C++ 架构搭建（UI、Robot、Controller 分离）。

已实现 XML 模型配置（添加阻尼）。

已实现核心功能：约束监测与违规冻结、可视化警告、键盘交互重置。

已建立通用控制器接口 RobotController，但具体控制算法（如 QP/PID）尚未填充。

2. 关键文件结构

A. 模型文件

robot.xml:

定义了 7 自由度机器人（躯干 3 DoF + 双腿各 2 DoF）。

关键修改: 在腿部关节（hip, knee）添加了 damping="0.1"。

惯量: 腿部惯量通过 <inertial> 显式定义，验证符合 $I=\frac{1}{12}ml^2$。

B. C++ 源码架构

main.cpp (入口)

负责初始化 MuJoCo 和 GLFW。

包含主仿真循环（Physics Loop + Rendering Loop）。

关键逻辑: 检测到 robot.getWarningMessage() 非空时，主动 break 物理循环以实现“画面冻结”并渲染红色警告。

当前模式：调用 robot.stand(...)。

BipedRobot.h/cpp (机器人抽象类)

持有 mjModel* 和 mjData*。

checkConstraints(): 每一帧调用，监测角度、角速度、力矩是否超标。

坐标系修正: 代码中对读取的 qpos, qvel, actuator_force 进行了取反 (-) 操作，以匹配题目要求的“逆时针为正”（对应 MuJoCo Y轴负向）。

状态管理: resetToKeyframe (空格键) 和 resetToState (R键，自定义空中坐姿)。

动作接口: freeFall(), stand(), walk()。

RobotController.h/cpp (控制器类)

定位: 纯算法层，与机器人物理状态解耦。

当前接口: computeStandControl(...)。

当前状态: 返回全 0 力矩（待实现具体算法）。

SimulationUI.h/cpp (工具类)

处理 GLFW 回调（键盘、鼠标）。

利用 glfwGetWindowUserPointer 获取 BipedRobot 实例进行交互。

CMakeLists.txt

依赖: MuJoCo, GLFW, Eigen3, OSQP, OsqpEigen。

包含 RPATH 设置以处理动态库路径。

3. 核心约束参数 (Constraints)

若违反以下任意条件，仿真必须立即冻结并在屏幕右上角显示红色警告：

关节

角度范围 (Angle)

速度上限 (Vel)

力矩上限 (Torque)

Hip

$-120^\circ \le q \le 30^\circ$

$

\dot{q}

Knee

$0^\circ \le q \le 160^\circ$

$

\dot{q}

注：代码中使用弧度 (Rad) 进行计算。


5. 环境配置注意

MuJoCo 版本: 3.3.6 (MacOS dylib 路径已在 CMake 中硬编码，迁移环境需注意修改 MUJOCO_FRAMEWORKS 路径)。

C++ 标准: C++17。