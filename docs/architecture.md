# mvp_rmcs 目标架构 — 现代架构 + RMCS 内容

> 状态:设计 v0.3(2026-11-02)| 范围:哨兵(舵轮底盘、偏心双 Yaw 云台、发射、火控与自瞄的控制侧、雷达 SLAM、导航决策;视觉除外),从单电机起步逐步长成整车
> 分阶段实施计划:`01_architecture.md` / `02_control.md` / `03_lidar_slam.md` / `04_navigation.md`
> 一句话:**架构取现代工程(ros2_control + 标准 ROS2),内容取 RMCS;从 C 板 + 一个 3508 起步,遇到工程问题再引入对应机制。**

---

## 0. 修订说明(v0.2 → v0.3)

| # | v0.2 的做法 | v0.3 的调整 |
|---|---|---|
| 1 | 运动控制复用 ros2_controllers(pid_controller、底盘控制器),导航和决策复用 Nav2、行为树 | **内容全部取自 RMCS**:设备协议、PID、云台、底盘、发射、导航(rmcs-navigation)、决策(Lua)、自瞄、裁判。ros2_control 只当架构宿主用 |
| 2 | 用 mock 硬件和仿真开发 | **删除**。直接用真机(C 板 + 电机),RMCS 的硬件代码原样复用 |
| 3 | 路线图一上来就铺满安全、测试、CI、实时、可观测 | **按 MVP 方式生长**。阶段 0 只搭最小骨架;每阶段只加一个情景;工程细节进入"触发式清单"(§7),出现对应现象才引入 |
| 4 | 把 PID 改成显式 dt | **保留 RMCS 的按拍语义**,`update_rate` 锁定 1 kHz。只有在需要不同频率时才引入 dt(列入触发清单) |

v0.2 的这些判断保留不变:
- 控制环宿主用 ros2_control,不自研执行器,也不写自研装配框架;
- 环内和环外严格分界;
- 硬件按"板"来写,拓扑写进 URDF;
- 串级要区分测量值和设定值(state / reference 是两类接口)。

v0.1 和 v0.2 的原文备份在 `/tmp/architecture.v0.{1,2}.md`。

---

## 1. 定位:架构和内容分开取

| 层面 | 取自 | 包括 |
|---|---|---|
| **架构** | 现代工程:ros2_control + 标准 ROS2 | 执行模型(read → update → write)、组件契约(接口声明与激活期校验)、装配(URDF + controllers.yaml + launch)、生命周期、环内与环外的边界 |
| **内容** | RMCS | 设备协议(DJI 电机、DR16、BMI088、超电、裁判)、控制算法(PID、ADRC、滤波、QCP)、控制逻辑(云台、底盘、发射、功率)、导航与决策(rmcs-navigation、Lua)、定位建图(point_lio、local-map、relocation)、自瞄(rmcs_auto_aim_v2) |
| **工具** | ros2_control 生态 | joint_state_broadcaster(观察状态)、forward_command_controller(开环调试)。它们只是调试和观察手段,不承担控制内容 |

**不再使用 RMCS 的这些架构部件:**
- `rmcs_executor` 执行器;
- `Component` / `Input/OutputInterface`,以及 partner component;
- `predefined_msg_provider`;
- `main.cpp` 的 pluginlib 装配方式;
- "每个车型手写一个硬件类"的写法。注意,这些硬件类里的协议和打包逻辑仍然作为内容复用。

---

## 2. 目标架构(终局形态,不是第一步)

```
 环外(异步,普通 ROS2 节点)
 ┌──────────────────────────────────────────────────────────────────┐
 │ rmcs-navigation(Lua 决策 + MotionFsm + Nav2 客户端)              │
 │ point_lio / rmcs-local-map / rmcs_relocation / Nav2 栈(原样)      │
 │ rmcs_auto_aim_v2_runtime(原样,独立进程)                          │
 └──────────┬──────────────────────────────────────────▲─────────────┘
   参考量(取最新值,超时失效)│                           │ 环内状态(广播器)
 ═══════════▼═══════ 环边界:话题只出现在这里 ══════════════╧═══════════
 环内(ros2_control_node,一个线程,1 kHz)
 ┌──────────────────────────────────────────────────────────────────┐
 │ read() ──► 控制器(controller_manager 按依赖排序)──► write()       │
 │            RMCS 内容:云台 / 底盘 / 发射 / 功率 / PID                │
 │            自瞄桥接(共享内存,沿用 RMCS 做法)                     │
 └──────────┬──────────────────────────────────────────▲─────────────┘
 ┌──────────▼──────────────────────────────────────────┴─────────────┐
 │ 硬件组件:每块板一个 SystemInterface(内部是 librmcs::agent::CBoard)│
 │   设备类(RMCS 原样):DjiMotor / Dr16 / Bmi088 / Supercap / 裁判解析 │
 └──────────┬───────────────────────────────────────────────────────┘
            ▼ USB ── C 板 ── CAN ── C620/GM6020 电调(电调内部跑电流环)
```

层次规则:

1. **依赖单向**:bringup → 控制器 / 硬件 → RMCS 内容头文件。内容头文件不依赖 ROS。
2. **控制器只认接口名**,不认设备型号;硬件只认接口名,不认控制策略。
3. **环外数据进环只有一条路**:控制器的参考量入口,也就是一个话题,控制器只取它的最新值。

---

## 3. 第一天就定下的架构约束

原则:**改起来贵的现在定,改起来便宜的等问题出现再加**(便宜的那些见 §7)。

| # | 约束 | 为什么现在就定 | 阶段 0 里怎么体现 |
|---|---|---|---|
| C1 | **控制环宿主用 ros2_control** | 组件的编程模型(写成 `update()`,还是写成订阅回调)是最贵的改动,事后切换等于重写所有组件 | 硬件写成 SystemInterface,控制器写成 controller 插件 |
| C2 | **环内不走话题** | 一旦控制链拆成多跳话题,同拍顺序就只能靠 stamp 配对去补(现在阶段 6 的代码就是这样) | 速度环在 CM 线程里直接读写共享接口 |
| C3 | **硬件按板写,设备类和框架解耦** | 同一个协议实现要在所有车型上复用 | `CBoardSystem` 内部用 RMCS 的 `DjiMotor` |
| C4 | **内容头文件不 include ROS** | 将来要单测、要换宿主时,不用再去剥离 ROS | `pid_calculator.hpp` 删掉 `#include <rclcpp/node.hpp>` |
| C5 | **单位用 SI,命名统一** | 改名会牵动所有配置 | 关节 `lab_motor_0`;rad、rad/s、N·m(RMCS 本来就是 SI) |
| C6 | **依赖版本锁定** | 硬件协议库和固件版本必须匹配 | ROS 锁 Jazzy;librmcs 锁 v3.1.0,URL 和 SHA256 与 RMCS 的 `rmcs_core/CMakeLists.txt` 第 16–24 行一致 |
| C7 | **最小安全底线**(真机必需,是"工程细节后加"的唯一例外) | 第一次上电就有失控风险 | 见 §5.6 |

**C1 的取舍**,完整对比见 v0.2 §5,这里只保留结论:

- 选 ros2_control 的理由:它自带同拍顺序(链式控制器拓扑排序)、接口独占校验、生命周期和实时线程;
- 代价一:环内只能传标量,用 §4.3 的类型映射来解决;
- 代价二:即使不需要几何,也要写一份最小 URDF。

---

## 4. RMCS 内容迁移:内容原样搬,只换外壳

### 4.1 硬件内容

| RMCS 文件 | 和 rmcs_executor 的耦合 | 迁移动作 |
|---|---|---|
| `hardware/device/dji_motor.hpp` | 只有构造函数里的 register_input/output,加上 5 处 `*_output_` 写入 | 去掉这两处耦合,改成普通 getter 和 setter。解码、换算、`generate_command`(NaN → 0、按 max_torque 限幅)**原样保留** |
| `hardware/device/can_packet.hpp`、`bmi088.hpp` | 无 | 原样复制 |
| `hardware/device/dr16.hpp` | 构造函数注册 10 个 `/remote/*` 输出 | 18 字节解码部分原样保留,注册部分删掉 |
| `supercap.hpp`、`lk_motor.hpp` | 注册输入输出(lk_motor 还 include 了 rclcpp) | 用到时再剥离,协议部分保留 |
| `rmcs_utility/{endian_promise, ring_buffer, crc}` | 无(纯 std) | 被引用时一并带上 |
| `omni_infantry.cpp` 这类车型硬件类 | 本身就是一个 executor 组件 | **不搬类,搬逻辑**:接收回调里的 CAN ID 分发、`command_update()` 的帧打包,挪进 `CBoardSystem` 的回调和 `write()` |
| librmcs | 外部库,FetchContent | 用和 RMCS 相同的固定版本 |

RMCS 的线程模型可以直接沿用:
- librmcs 的 USB 事件线程在回调里调 `store_status()`,写入 `std::atomic<CanPacket8>`;
- 控制线程在 `read()` 里调 `update_status()`,在 `write()` 里调 `start_transmit()`;
- 这和 RMCS 中 hardware `update()` → 控制器 → `command_update()` 的顺序完全对应。

### 4.2 控制器迁移配方

| RMCS 写法 | ros2_control 写法 |
|---|---|
| 构造函数里 `register_input(name, …)` | `state_interface_configuration()` 返回接口名;如果数据来自另一个控制器,就读它导出的接口 |
| `register_output(name, …)` | 写给硬件或下游控制器的,放进 `command_interface_configuration()`;给别的控制器读的,用 `export_state_interfaces()` |
| `get_parameter(...)`(构造时读取) | `on_configure` 里读取;参数名尽量和 RMCS 的 yaml 保持一致 |
| `update()` 函数体 | `update(time, period)`,**函数体原样搬**。`*input_` 换成读 state handle,`*output_ = x` 换成写 command handle |
| `/predefined/timestamp` | `update()` 的 `time` 参数 |
| partner component(把状态和命令拆成两个组件) | 不再需要,read 和 write 由框架分开调用 |
| SmartInput 里绑一个常量(`setpoint: 5.0`) | 保留同样的语义:参数是数字就当常量,是名字就当接口 |

### 4.3 端口类型映射

| RMCS 端口类型 | 环内表示 |
|---|---|
| `double` | double 接口 |
| `bool`、`int64_t`(热量等) | bool / 整型接口。Jazzy 4.x 的 handle 已支持 bool 和最高 32 位整型,具体以实装版本为准;不支持时用 double 承载 |
| `Switch` / `ChassisMode` / `GimbalMode` / `ShootMode` / `GameStage`(uint8 枚举)、`Mouse` / `Keyboard`(位域) | uint8 / uint16 接口。`rmcs_msgs` 的这些头文件是纯头文件,原样复用 |
| `Eigen::Vector2d/3d`、`BaseLink::DirectionVector` | 拆成多个标量接口(例如 `chassis/vx`、`chassis/vy`、`chassis/wz`) |
| `Tf`(fast_tf) | 不在接口上传。控制器内部用 `rmcs_description` 根据关节角计算 |
| `steady_clock::time_point` | 用 `update()` 的 time 参数 |
| `SerialInterface`(`std::function`) | 不出硬件组件,串口由板子的 SystemInterface 自己持有 |

### 4.4 环外内容

| RMCS 内容 | 现状 | 新架构里的位置 |
|---|---|---|
| point_lio、rmcs-local-map、rmcs_relocation、Nav2 栈 | 已经是独立的 ROS2 节点 | **原样复用** |
| rmcs_auto_aim_v2_runtime | 独立进程,通过 POSIX 共享内存和环内通信 | **原样复用** |
| `AutoAimComponent` | executor 组件:发出云台和相机的上下文,收回瞄准指令(丢弃超过 100 ms 的指令) | 改成环内的**桥接控制器**,共享内存保持不变,结果作为状态接口导出给云台和拨弹控制器 |
| `rmcs::navigation::Navigation` | executor 组件 + Node:Lua 决策约 90 Hz,跑在 1 kHz 线程上 | **拆成两半**。Lua 决策、MotionFsm、Nav2 客户端、重定位客户端移到环外节点;环内只留底盘和云台的参考量入口(带超时,RMCS 原来是 500 ms)。好处是 Lua 不再跑在实时线程上 |
| 裁判 `Status` | executor 组件,在 update 里解析串口帧 | 解析逻辑放进板子的 SystemInterface(串口归板子)。控制要用的字段(功率上限、缓冲能量、热量、比赛阶段)导出为状态接口;全量数据通过广播发到环外,供决策使用 |
| 裁判 `Command` / `Ui` / `Interaction` | executor 组件 | 写方向的归属待定(§9) |
| ValueBroadcaster / TfBroadcaster | 在 `sentry.yaml` 中已被注释掉,所以目前环内数据完全出不了环 | 改用广播器和 robot_state_publisher,到阶段 7 需要时再加 |

### 4.5 迁移时顺手修的 RMCS 问题(已核对源码)

| 位置 | 问题 |
|---|---|
| `rmcs-navigation/src/cxx/navigation.cc` 第 183–188 行、第 273 行 | `latest_cmd_vel` 在 ROS 回调里写、在控制线程里读,没有任何同步 |
| `hardware/omni_infantry.cpp` 第 231–238 行 | `calibrate_zero_point()` 在 ROS 回调里修改电机内部状态,和控制线程并发 |
| `controller/gimbal/dual_yaw_controller.cpp` 第 48–49 行 | 同一个输出接口被注册了两次。按 `register_output` 的检查,这个组件一构造就会抛异常;目前没有任何 yaml 使用它 |
| `controller/pid/pid_calculator.hpp` 第 32–34 行 | 输入是 NaN 时直接返回,但不清 `last_err_`,恢复后的第一拍 D 项会用到过期误差 |
| `friction_wheel_controller.cpp` 第 60 行等多处 | 写死了 1 kHz,迁移时要么保持 1 kHz,要么改用 `period` |

---

## 5. 阶段 0:最小骨架(C 板 + 一个 M3508 + 速度环)

目标:用最少的部件搭出整套架构骨架,让一个 3508 稳定跟踪一个速度目标。
对照:RMCS 根目录的 `motor_control_lab.md` 是同一件事在 RMCS 架构下的设计。本阶段的数据流、CAN 约定和调试顺序都与它对齐,区别只在宿主框架。

> 落地时用的是哨兵的真实电机:底板 C 板 can1 上的左后轮(关节 `chassis_wheel_lb`,反馈 0x201,减速比 11,reversed)。下文的 `lab_motor_0` 只是通用示意。具体步骤见 `01_architecture.md` 的阶段 0–3。

### 5.1 范围

三个包,只放必需的文件:

```
mvp_rmcs/src/
  mvp_hardware/
    include/mvp_hardware/device/   dji_motor.hpp、can_packet.hpp(RMCS 原样,去掉 executor 耦合)
                                   及其引用的 rmcs_utility 纯头文件
    src/c_board_system.cpp         SystemInterface;读/写拆分照搬 omni_infantry 的结构
    mvp_hardware.xml               pluginlib 导出
    CMakeLists.txt                 FetchContent librmcs v3.1.0(URL 和 SHA256 同 RMCS)
  mvp_controllers/
    include/mvp_controllers/pid/   pid_calculator.hpp(RMCS 原样,删掉 rclcpp include 和 make_pid_calculator)
    src/pid_controller.cpp         RMCS PidController 的 update 搬过来
    mvp_controllers.xml
  mvp_bringup/
    urdf/motor_lab.urdf.xacro      一个关节 + <ros2_control> 段
    config/motor_lab/controllers.yaml
    launch/bringup.launch.py
```

现有的 `mvp_core` / `mvp_msgs`(话题环)是阶段 6 的学习产物,本阶段退役。它们目前还没有纳入 git,建议先提交一次,留在历史里。

### 5.2 数据流

```
3508 编码器 → C620 反馈帧 0x201 → C 板 → USB
  → [librmcs 事件线程] can1_receive_callback → DjiMotor::store_status(原子写)
  → [CM 线程] read():update_status() → lab_motor_0/{position, velocity, effort} 状态接口
  → [CM 线程] update():PidController(setpoint - velocity)→ lab_motor_0/effort 命令接口
  → [CM 线程] write():generate_command() → 0x200 帧第 1 槽 → start_transmit()
  → C 板 → C620 → 3508
```

### 5.3 配置样例(示意)

URDF 里的 `<ros2_control>` 段:

```xml
<ros2_control name="c_board" type="system">
  <hardware>
    <plugin>mvp_hardware/CBoardSystem</plugin>
    <param name="board_serial"></param>          <!-- 空字符串表示取第一块 C 板 -->
  </hardware>
  <joint name="lab_motor_0">
    <param name="can_bus">1</param>
    <param name="motor_id">1</param>              <!-- 反馈 0x201;命令 0x200 第 1 槽 -->
    <param name="motor_type">m3508</param>        <!-- 减速比不写就用 DjiMotor 默认值 3591/187 -->
    <param name="reversed">false</param>
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

`controllers.yaml`(参数名沿用 RMCS PidController,数值取 `motor_control_lab.md` 的安全起点):

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000            # RMCS 的 PID 增益按拍定义,频率锁定 1 kHz
    joint_state_broadcaster: {type: joint_state_broadcaster/JointStateBroadcaster}
    lab_motor_0_effort_forward: {type: forward_command_controller/ForwardCommandController}
    lab_motor_0_velocity_pid: {type: mvp_controllers/PidController}

lab_motor_0_effort_forward:      # 调试工具:开环小力矩,用来确认方向
  ros__parameters: {joints: [lab_motor_0], interface_name: effort}

lab_motor_0_velocity_pid:
  ros__parameters:
    measurement: lab_motor_0/velocity   # RMCS 里是 /lab/motor/0/velocity
    control: lab_motor_0/effort         # RMCS 里是 /lab/motor/0/control_torque
    kp: 0.02
    ki: 0.0
    kd: 0.0
    output_min: -0.3
    output_max: 0.3
```

- **setpoint 来源**:阶段 0 从话题 `~/setpoint`(`std_msgs/Float64`)读,内部用一个 `std::atomic<double>` 交给控制线程,初值为 0。
- **模式切换**:`motor_control_lab.md` 里的三种模式 `disabled` / `open_loop_torque` / `velocity`,在这里就是"激活哪个控制器"。两个控制器都要占用 `lab_motor_0/effort`,controller_manager 不允许它们同时激活,模式互斥由框架保证。

### 5.4 launch(薄壳)

1. robot_state_publisher 发布 robot_description;
2. 启动 ros2_control_node,加载 `controllers.yaml`;
3. spawner 激活 joint_state_broadcaster;
4. 另外两个控制器用 `spawner --inactive` 只加载、不激活。

### 5.5 环境前提

- **依赖**:ros2_control、ros2_controllers、xacro、libusb-1.0-0-dev 都写进 package.xml,由 rosdep 安装。现在的 Dockerfile 基于 ros-base,本身不带这些包。
- **USB 透传**:需要把 C 板(USB VID:PID `a11c:d401`)透传进容器。RMCS 用的是 `privileged: true`,我们可以沿用;另外容器里是非 root 用户,还要在宿主机加一条 udev 规则,让这个设备对普通用户可写。

### 5.6 最小安全底线(只有这些)

| 机制 | 来源 | 代价 |
|---|---|---|
| 命令是 NaN 时发 0 电流 | RMCS `dji_motor.hpp` 第 185–187 行,原样保留 | 0 |
| 命令接口初值设为 NaN;控制器被停用、接口被释放时(`perform_command_mode_switch`),把对应命令写回 NaN | 新增约 10 行。原因:ros2_control 停用控制器后**不会**清掉命令值 | 约 10 行 |
| 硬件 `on_deactivate` 时发一帧全 0 | 新增 | 几行 |
| 输出限幅 | RMCS PidController 的 `output_min/max`,加上 DjiMotor 按 max_torque 限幅 | 0 |
| 第一次上电时空载并固定电机 | `motor_control_lab.md` §3 | 操作规程 |

### 5.7 上电步骤(与 `motor_control_lab.md` §9 对齐)

| 步 | 做什么 | 验证 |
|---|---|---|
| 0a 只读反馈 | 只激活 joint_state_broadcaster | 手拨电机,`ros2 topic echo /joint_states` 里 position 和 velocity 跟着变;`ros2 control list_hardware_interfaces` 能看到全部接口 |
| 0b 开环小力矩 | 激活 `lab_motor_0_effort_forward`,然后执行 `ros2 topic pub --once /lab_motor_0_effort_forward/commands std_msgs/msg/Float64MultiArray '{data: [0.02]}'` | 电机缓慢转动。方向反了就改 URDF 里的 `reversed`,不要在 PID 里加负号 |
| 0c 速度 P | 执行 `ros2 control switch_controllers --deactivate lab_motor_0_effort_forward --activate lab_motor_0_velocity_pid`,再往 `/lab_motor_0_velocity_pid/setpoint` 发 1.0 | velocity 稳定在约 1 rad/s。顺带验证:尝试同时激活两个控制器,应当被拒绝 |
| 0d 加 I / D | 按 `motor_control_lab.md` §9.4 的顺序调参 | 稳态误差消除 |

### 5.8 刻意不做的事

以下内容进入 §7 的触发清单,本阶段不做:
- 单元测试和 CI;
- 实时优先级和绑核;
- 参数校验库;
- 遥控器急停;
- 反馈超时、温度保护;
- 在线调参;
- 诊断;
- 多电机打包;
- 串级;
- 独立的算法包。

### 5.9 检查点

- **新增约束**:控制环在 CM 线程里运行;硬件和控制器通过接口名连接;模式等于"激活了哪个控制器"。
- **解决的问题**:remap 拼错静默不闭合的问题消失(接口名写错时控制器无法激活);同拍语义不再需要 stamp 配对。
- **代价**:要写最小 URDF;controller 插件有样板代码;需要处理 USB 透传。

---

## 6. 生长路线

v0.3 版的 9 行路线表已经细化为四份分阶段计划(每个阶段都写成"需求 → 方法选择 + 原因 → 具体实现 → 验证"):

| 计划 | 内容 | 阶段 |
|---|---|---|
| `01_architecture.md` | 架构与硬件:C 板 + 3508 → 写通道与安全底线 → PID → 控制器链 → GM6020 → 底板 8 电机拓扑数据化 → 标定分离 → DR16 → 急停收敛到硬件层 → IMU → LK 电机 → 顶板 → 云台 IMU 板 → URDF/TF → 裁判串口 → 超电 → 环内外边界 → 整车 bringup | 0–18 |
| `02_control.md` | 控制:舵轮底盘 A1–A19、偏心双 Yaw 云台 B1–B12、发射 C1–C6、火控与自瞄控制侧 D1–D10(先接上 RMCS 现有进程,再在 `Armor3d` 切分线切开) | 47 |
| `03_lidar_slam.md` | 雷达:驱动 → **安装位置与坐标系约定** → point_lio → 录包回放 → PCD 建图 → 二维栅格(新增)→ 初始重定位 → 开机自动重定位(新增)→ 局部地图 → 漂移修正 | 0–9 |
| `04_navigation.md` | 导航决策:接口约定 → `cmd_vel` 开车 → 纯软件环境 → 最小 Nav2 → 局部代价地图 → 全局地图 → 调参 → 目标 API 节点 → 决策上下文 → Lua 运行时节点 → 意图 → MotionFsm 节点 → 云台扫描 → 重定位时机 → 进程管理 | 0–14 |

跨计划的关键依赖:

- 02 的底盘部分从 01 阶段 6 之后就可以开始;
- 03 阶段 1 决定 `base_link` 的语义,会回头影响 01 阶段 14 的 URDF 和 02 的 A17;
- 02 的 A17、B10 依赖 04。

## 7. 工程细节:出现问题再引入

下表只列触发条件,不排日程。"常见于"一栏是预计最早遇到的阶段,仅供参考。

| 现象(触发信号) | 引入什么 | RMCS 现状 / 参考 | 常见于 |
|---|---|---|---|
| 调参时看不到曲线 | CM introspection 加 PlotJuggler;控制器状态用 RealtimePublisher 发出 | ValueBroadcaster | 0d |
| 外环切换或模式切换时有冲击 | `on_activate` 里调 `PidCalculator::reset()` | `motor_control_lab.md` §10 | 1 |
| 遥控器断开后电机保持最后的指令 | DR16 断连超时 | RMCS 没有(`dr16.hpp`) | 2 |
| 新写的控制器漏了急停判断 | 在硬件层统一截断(阶段 2 已决策,此处是兜底) | 11 处分散判断 | 2 |
| 电机反馈断了,控制器还在出力 | 每个设备记录收帧计数或时间,超时置 NaN | 只有 deformable 车型有 | 3 |
| CM 报 overrun,电机异响或抖动 | SCHED_FIFO(容器加 `cap_add: SYS_NICE` 和 rtprio / memlock ulimits)、`cpu_affinity`、`lock_memory`、lowlatency 内核 | — | 3–5 |
| 堵转后电机发烫 | 导出温度状态接口,加阈值保护 | `dji_motor.hpp` 已解码温度,但没有输出 | 3 |
| 参数拼错,静默用了默认值 | 关键参数不给默认值(RMCS 的 kp/ki/kd 本来就是必填);规模变大后再上 generate_parameter_library | `pid_controller.cpp` 第 22–24 行 | 任意 |
| 配置写错,启动之后才发现 | spawner 失败时让 launch 整体退出 | — | 任意 |
| 改了算法导致回归 | 给内容头文件写 gtest;这时再把它们拆成独立的算法包 | — | 4–5 |
| 多人协作后编译不过 | CI:colcon build / test、clang-format | — | 任意 |
| 需要在线调参 | 参数回调写入实时安全的缓冲区 | — | 1–5 |
| 某个控制器需要不同的频率 | PID 改为显式 dt;排查写死 1 kHz 的地方(§4.5) | 增益按拍定义 | 视需要 |
| USB 掉线导致进程崩溃 | launch 里 respawn;或者给 librmcs 加错误路径 | librmcs 在设备拔出时调 `std::terminate()`;RMCS 靠 `respawn=True` | 3+ |
| 零点标定和控制线程抢数据 | 标定请求通过原子标志交给 CM 线程执行 | §4.5 的数据竞争 | 5 |
| 环外节点多了,启停顺序乱 | lifecycle 节点 + `nav2_lifecycle_manager` | — | 7 |
| 上层断连后底盘还在跑 | 参考量超时(底盘 500 ms、自瞄 100 ms,沿用 RMCS 的值) | `rmcs-navigation/src/cxx/component.cc` 第 22 行、`rmcs_auto_aim_v2/src/component.cpp` 第 117 行 | 4、7、8 |
| 上位机抖动怎么调都下不来 | 评估把内环下放到 MCU | — | — |

---

## 8. 约定

| 对象 | 规则 | 例子 |
|---|---|---|
| 关节 | `<机构>_<部件>`,实验台用 `lab_motor_<n>` | `lab_motor_0`、`gimbal_yaw`、`chassis_wheel_lf`、`shooter_friction_l` |
| 接口 | position / velocity / effort 用标准名;RMCS 的 `control_torque` 对应命令接口 `effort`,`angle` 对应 `position`,`torque` 对应状态接口 `effort` | `gimbal_yaw/effort` |
| 控制器 | `<关节或机构>_<量>_<类型>` | `lab_motor_0_velocity_pid` |
| 单位 | SI,和 RMCS 一致 | rad、rad/s、N·m |
| 来源标注 | 从 RMCS 复制的文件,头部注明原路径;修改处加注释 | `// 来源:rmcs_core/src/hardware/device/dji_motor.hpp` |
| 目录 | `mvp_hardware`(板级组件 + 设备类)、`mvp_controllers`(控制器 + 算法头文件)、`mvp_bringup`(URDF + 配置 + launch)。环外包和独立的算法包,等需要时再加 | — |

---

## 9. 决策记录与待决问题

**决策记录**

| # | 决策 | 否决的方案 | 理由 |
|---|---|---|---|
| D1 | 控制环宿主用 ros2_control | 话题事件驱动;自研执行器 | §3 C1 |
| D2 | 内容取 RMCS;ros2_control 生态只用调试和观察工具 | 复用 ros2_controllers 和 Nav2 的控制内容 | RMCS 内容已经在真车上验证过,是自己的积累 |
| D3 | 真机起步,不做 mock | mock 硬件、仿真 | 硬件现成;RMCS 的硬件代码可以直接复用 |
| D4 | MVP 方式生长:第一天只定改起来贵的约束,工程细节按触发引入 | 一次写全 | 每个机制都对应一个真实出现过的问题 |
| D5 | 保留 RMCS 按拍的 PID,锁定 1 kHz | 一开始就 dt 化 | 内容原样复用;需要多频率时再改 |
| D6 | 急停统一放在硬件层(阶段 2) | 每个控制器各自判断 | RMCS 有 11 处分散判断,漏写一处就不安全 |
| D7 | 自瞄先保留共享内存通道 | 一开始就改成话题 | RMCS 已经验证过它的延迟;是否改话题,等调试需求出现再定 |

**待决问题**

- apt 版 ros2_control 是否支持整型和 bool 句柄(阶段 2 的拨杆会用到)?装包后核对;不支持就用 double 承载。
- 裁判"写"方向(UI、交互、哨兵决策指令)放在哪里:硬件组件内部的发送队列,还是环外节点?阶段 6 或 7 决定。
- 导航的环内入口:直接作为底盘和云台控制器的参考量,还是做成一个独立的"上层指令"控制器?阶段 7 决定。
- 多块板时,各 SystemInterface 在同一个 CM 里的 read / write 顺序和同步方式:阶段 9 决定。
