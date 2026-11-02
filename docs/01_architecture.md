# Plan 01 — 架构搭建(哨兵 · ros2_control)

> 用法:把本文件全文发给 AI 助手(编码 Agent)。必须严格按阶段推进:
> - 每个阶段只解决一个问题,改动尽量小;
> - 每个阶段先写"需求 → 方法选择 + 原因",再写代码;
> - 阶段结束后停在检查点,等我验证通过再继续。

## 角色

你是我的结对程序员。工作区是 `/home/yuyuyu/Life/项目/RM机器人/mvp_rmcs`(ROS2 Jazzy)。目标是把哨兵从"一块 C 板 + 一个 3508"开始,一步步搭成整车硬件,并建好环内外接口。

- **架构**用 ros2_control;
- **内容**(设备协议、板子收发逻辑)取自 `../RMCS/rmcs_ws/src`。

## 背景

- 总设计见 `docs/architecture.md`(v0.3)。本计划负责落地其中的硬件层和架构骨架。
- 范围只包括哨兵:舵轮底盘、偏心双 Yaw 云台、发射机构,以及它们用到的遥控器、IMU、裁判系统、超电。
- 后续计划:`02_control.md`(控制算法,依赖本计划的硬件阶段)、`03_lidar_slam.md`、`04_navigation.md`。
- RMCS 的硬件代码集中在 `rmcs_core/src/hardware/sentry.cpp` 和 `hardware/device/`。librmcs 由 `rmcs_core/CMakeLists.txt` 第 16–24 行通过 FetchContent 拉取 v3.1.0,带 SHA256 校验。

## 铁律

1. **架构取现代工程,内容取 RMCS。** RMCS 代码原样搬过来,只换外壳;每个文件头注明来源路径。
2. **每个阶段只解决一个问题。** 不顺手加料。
3. **工程细节按需引入。** 实时调度、测试、CI、诊断、参数校验等,只在出现对应现象时才加(见 `architecture.md` §7)。唯一的例外是阶段 2 的最小安全底线。
4. **每个阶段结束都能运行、能验证。** 真机上电前,先确认电机空载且已固定。
5. **每个阶段都按这个格式写:** 需求(问题)→ 方法选择 + 原因 → 具体实现 → 验证 → 检查点。

## 哨兵硬件总表(取自 RMCS)

| 板 | librmcs 类型 | 序列号参数(当前值) | 设备 |
|---|---|---|---|
| 底板 BottomBoard | `CBoard`,跳过固件版本检查(`sentry.cpp` 第 333 行) | `board_serial_bottom_board`(`d4-2184`) | **can1**:左前/左后轮 M3508、左前/左后舵 GM6020、大 Yaw LK MG6012<br>**can2**:右后/右前轮、右后/右前舵、超电<br>**dbus**:DR16<br>**uart1**:裁判系统<br>**板载 BMI088**:底盘陀螺 |
| 顶板 TopBoard | `RmcsBoardLite` | `board_serial_top_board`(`af-da30`) | **can0**:左右摩擦轮 M3508、拨弹 M3508<br>**can2**:pitch LK MG4010<br>**can3**:小 Yaw LK MG4010<br>**板载 BMI088**:装在大 Yaw 上 |
| 云台板 GimbalBoard | `CBoard` | `board_serial_gimbal_board`(`d4-1d2b`) | 只有板载 BMI088,装在 pitch 上。云台控制用的就是这块 IMU |

librmcs 线程模型(沿用,不改):
- 每块板有自己的 USB 事件线程,接收回调都在这个线程里执行,只做原子写入(`store_status`)。
- 控制线程负责 `update_status` 和 `start_transmit()`。
- 设备拔出时 librmcs 会直接调用 `std::terminate()`,没有给上层报错的机会。

---

## 阶段 0 — 环境:容器能看到板子,能编译 librmcs

- **需求(问题)**:现在的容器基于 ros-base,既没有 ros2_control,也看不到 USB 设备,真机代码无从下手。
- **方法选择 + 原因**:
  - 依赖写进各包的 package.xml,由 rosdep 安装。这样 Dockerfile 不写死包名,换机器时可以重放。
  - USB 透传沿用 RMCS 的 `privileged: true`,这是最省事的做法。另外在宿主机加一条 udev 规则,给 VID `a11c` 的设备开放权限,因为容器里跑的是非 root 用户。
  - 不选只映射 `/dev/bus/usb`:板子重新插拔后设备号会变,还得重启容器。
- **具体实现**:
  - 建三个空包:`mvp_hardware`、`mvp_controllers`、`mvp_bringup`。
  - `mvp_hardware` 的 CMakeLists 照抄 RMCS 的 FetchContent 段(同一个 URL 和 SHA256,`BUILD_STATIC_LIBRMCS ON`,链接 `librmcs-sdk`),并依赖 libusb-1.0。
  - 改 compose 文件和 udev 规则。
  - 现有的 `mvp_core` / `mvp_msgs` 是阶段 6 的学习产物,先提交到 git 再删除。
- **验证**:
  - 容器内 `lsusb` 能看到 `a11c:d401`(C 板)和 `a11c:a801`(RmcsBoardLite);
  - `colcon build` 通过;
  - 写一个 10 行的小程序构造 `librmcs::agent::CBoard{"d4-2184"}`,能连上板子且不抛异常。顺带确认底板是否仍然需要跳过版本检查。

## 阶段 1 — 一块板 + 一个电机:只读反馈

- **需求(问题)**:要先证明"板子 → 驱动 → ros2_control 状态接口"这条读通道是通的,写通道放到下一步。
- **方法选择 + 原因**:
  - 每块板写成一个 `hardware_interface::SystemInterface`。读写按板组织,和 RMCS 的 Board 类一一对应。
  - 设备类用 RMCS 的 `DjiMotor`,只去掉 executor 耦合:构造函数里的 `register_*` 调用,以及 5 处 `*_output_` 写入。
  - 不选"每个电机一个硬件组件":同一块板的帧要在一次 `start_transmit()` 里打包发出,拆开反而更复杂。
- **具体实现**:
  - `mvp_hardware/include/mvp_hardware/device/{dji_motor.hpp, can_packet.hpp}`,连同它引用的 `rmcs_utility` 纯头文件(`endian_promise.hpp` 等)。
  - `src/c_board_system.cpp`:
    - 私有继承 `librmcs::agent::CBoard`;
    - `can1_receive_callback` 按 CAN ID 分发给 `store_status`,写法照抄 `sentry.cpp` 第 452–503 行;
    - `read()` 里调 `update_status()`,再写入 position / velocity / effort 三个状态接口。
  - URDF 里只放一个关节 `chassis_wheel_lb`(哨兵左后轮,can1,反馈 0x201)。减速比按实物填写;装到车上时按 RMCS 的值 11、reversed。
  - launch 依次启动 robot_state_publisher、ros2_control_node,再用 spawner 激活 joint_state_broadcaster。
- **验证**:
  - `ros2 control list_hardware_interfaces` 能看到这三个接口;
  - 手拨电机时,`ros2 topic echo /joint_states` 里 position 和 velocity 跟着变化,单位是 rad 和 rad/s。

## 阶段 2 — 写通道 + 最小安全底线 + 开环定方向

- **需求(问题)**:要能给电机发力矩;同时这是第一次让真电机动起来,失控风险从这一步开始。
- **方法选择 + 原因**:
  - 开环测试用 `forward_command_controller`。它只是调试工具,不承担控制内容,不需要我们写代码。
  - 安全底线只加 4 条,前提是每条都不可或缺:
    1. **NaN 发 0 电流**:RMCS `dji_motor.hpp` 第 185–187 行已有,原样保留;
    2. **命令初值为 NaN**;
    3. **接口被释放时写回 NaN**:在 `perform_command_mode_switch` 里做。原因是 ros2_control 停用控制器后不会清掉命令值,不处理的话电机会一直保持最后一次的力矩;
    4. **`on_deactivate` 发一帧全 0**。
- **具体实现**:
  - 关节加上 `effort` 命令接口;
  - `write()` 用 `generate_command()` 拼 0x200 帧的第 1 槽,其余 3 槽填 `PaddingQuarter`,再调 `start_transmit().can1_transmit(...)`。
  - 写法参考 `motor_control_lab.md` §5 和 `sentry.cpp` 第 416–449 行。
- **验证**:
  1. 激活 forward 控制器,向 `/<name>/commands` 发 `0.02`,电机应缓慢转动;方向不对就改 URDF 里的 `reversed`。
  2. 停用 forward 控制器,电机必须立即无力。
  3. Ctrl-C 退出后,电机同样必须无力。

## 阶段 3 — 第一个控制器:RMCS PidController 速度环

- **需求(问题)**:要有一个真正的闭环控制器,用来验证"控制器插件 + 接口名接线"这一套机制。
- **方法选择 + 原因**:
  - 控制内容用 RMCS 的 `PidController` + `PidCalculator`。参数名(measurement / setpoint / control / kp / ki / kd / output_min / output_max)保持一致,以后 RMCS 的 yaml 可以直接对照。
  - setpoint 暂时从话题 `~/setpoint` 读,内部用一个 `std::atomic<double>` 交给控制线程。这是最简单的线程安全写法。
  - `update_rate` 锁定 1000。原因是 RMCS 的 PID 没有 dt,增益是按拍定义的(`pid_calculator.hpp` 第 32–50 行)。
- **具体实现**:
  - `mvp_controllers/include/.../pid/pid_calculator.hpp`:原样复制,只删掉 `#include <rclcpp/node.hpp>` 和 `make_pid_calculator`;
  - `src/pid_controller.cpp`:`update()` 的函数体从 `pid_controller.cpp` 第 38–41 行原样搬过来;
  - 在 `controllers.yaml` 里配置 forward 和 PID 两个控制器。
- **验证**:
  1. 用 `ros2 control switch_controllers` 从 forward 切到 PID;
  2. 向 setpoint 发 1.0,速度应稳定在约 1 rad/s;
  3. 尝试同时激活两个控制器,应被拒绝(两者都要占用同一个命令接口)。这说明"模式就是激活了哪个控制器",互斥由框架保证。

## 阶段 4 — 控制器链:在同一拍里组合控制器

- **需求(问题)**:串级控制和"上层控制器写下层控制器"在哨兵里到处都是:底盘模式 → 舵轮、摩擦轮逻辑 → 速度 PID。必须保证它们在同一拍内按顺序计算。
- **方法选择 + 原因**:
  - 用 ros2_control 的链式控制器:下游导出 reference 接口,上游把它当作命令接口写入,CM 按依赖关系排序。这与 RMCS executor 的拓扑排序作用相同,只是由框架提供。
  - 不选"话题串联":那样就回到了 stamp 配对的老问题。
- **具体实现**:
  - PidController 改为继承 `ChainableControllerInterface`,导出 `<name>/setpoint` 参考接口。没有被链接时,继续从话题读取。
  - 加一个外环 PID:`control: chassis_wheel_lb_velocity_pid/setpoint`,`measurement: chassis_wheel_lb/position`。
  - spawner 用 `--activate-as-group` 一次激活整条链。
- **验证**:
  - `ros2 control list_controllers` 能看到链式关系;
  - 0.5 rad 阶跃能收敛(对应 `motor_control_lab.md` §9.5);
  - 停用内环之前必须先停用外环,框架会强制这个顺序。

## 阶段 5 — 第二种 DJI 电机:GM6020 舵向

- **需求(问题)**:舵轮的舵向用 GM6020。它的反馈 ID、命令帧(0x1FE)、零点处理都和 3508 不同。
- **方法选择 + 原因**:
  - 电机型号和零点用 URDF 关节参数描述(`motor_type: gm6020`、`encoder_zero_point`)。设备类本身就支持多种型号(`DjiMotor::Config(Type)`),不需要另写一个类。
  - 命令帧按"总线 + 帧 ID"分组打包,为下一阶段的多电机做准备。
- **具体实现**:
  - 加关节 `chassis_steer_lb`(can1,反馈 0x205,0x1FE 帧第 1 槽,reversed,multi-turn);
  - `write()` 改为"收集同一组的槽,一组一帧"。
- **验证**:
  - 手转舵向,position 读数连续;回零后读数与零点一致;
  - 用 forward 控制器给小力矩,舵向转向正确。

## 阶段 6 — 硬件拓扑数据化:底板全部 8 个电机

- **需求(问题)**:4 个舵轮模块共 8 个电机,分布在两条 CAN 总线上。逐个写死在代码里既不可维护,也无法复用到别的车型。
- **方法选择 + 原因**:
  - 关节表完全由 URDF 给出:每个关节写 `can_bus`、`motor_id`、`motor_type`、`reduction_ratio`、`reversed`、`multi_turn`。硬件组件在 `on_init` 时根据这张表建立设备表和分发表。这是对 RMCS"每车型手写一个硬件类"的结构性替代。
  - 发送节奏沿用 RMCS:轮向帧和舵向帧隔拍交替发送(`sentry.cpp` 第 440–448 行),每组实际 500 Hz。原因可能是总线负载,有待确认。在测得问题之前不改。
- **具体实现**:
  - URDF 按下表补全关节(取自 `sentry.cpp`):

    | 模块 | 轮 M3508(总线, 反馈 ID, 命令帧槽) | 舵 GM6020(总线, 反馈 ID, 命令帧槽) |
    |---|---|---|
    | LF | can1, 0x202, 0x200 第 2 槽 | can1, 0x206, 0x1FE 第 2 槽 |
    | LB | can1, 0x201, 0x200 第 1 槽 | can1, 0x205, 0x1FE 第 1 槽 |
    | RB | can2, 0x202, 0x200 第 2 槽 | can2, 0x205, 0x1FE 第 1 槽 |
    | RF | can2, 0x204, 0x200 第 4 槽 | can2, 0x206, 0x1FE 第 2 槽 |

  - 轮向减速比 11,四个轮都 reversed;舵向减速比 1,reversed,multi-turn。
  - 关节命名为 `chassis_{wheel,steer}_{lf,lb,rb,rf}`。
- **验证**:
  - 8 个关节的状态都在 `/joint_states` 里;
  - 用 forward 控制器逐个给小力矩,每次只有对应的那个电机转。

## 阶段 7 — 标定数据分离:零点和单车数据

- **需求(问题)**:
  - 舵向零点会随装配变化,属于"这一台车"的数据,不应该和车型配置混在一起。
  - RMCS 里舵向零点参数名和实际模块错位了 180°(`sentry.cpp` 第 378–391 行:LF 读的是 `right_back_zero_point`),迁移时非常容易接错。
- **方法选择 + 原因**:
  - 零点放进 `config/sentry/calibration.yaml`,由 xacro 的 `load_yaml` 读入。参数直接按实际模块命名,在这里一次性纠正错位。
  - 标定流程沿用 RMCS 的思路:`/rmcs/service/robot_status` 会打印原始编码器值。这里改为多导出一个 `raw_encoder` 状态接口,直接从 `/joint_states` 之外的接口读取。
- **具体实现**:
  - 写 `calibration.yaml`。数值从 `sentry.yaml` 第 41–52 行按真实模块重新对应后填入;
  - 加 `raw_encoder` 状态接口。
- **验证**:
  - 四个舵向都摆正时,position 都接近 0;
  - 只改 `calibration.yaml` 就能调整零点,不需要重新编译。

## 阶段 8 — 遥控器 DR16:非关节状态接口

- **需求(问题)**:几乎所有控制器和安全逻辑都要用遥控器输入(拨杆、摇杆、键鼠),但它不是关节。
- **方法选择 + 原因**:
  - 在底板的 SystemInterface 里加 `<sensor name="remote">`,导出状态接口。
  - 拨杆使用 `rmcs_msgs::Switch` 的 uint8 编码,键盘用 uint16 位域,摇杆和鼠标用 double。Jazzy 4.x 的 handle 支持整型,以实际安装的版本为准;不支持时用 double 承载整数。
  - 解码逻辑用 `dr16.hpp` 第 49 行起的 18 字节解析,删掉构造函数里注册 10 个输出的部分。
- **具体实现**:
  - `dbus_receive_callback` → `store_status`;
  - `read()` 时写入 `remote/{joystick_right_x, joystick_right_y, joystick_left_x, joystick_left_y, switch_right, switch_left, mouse_x, mouse_y, mouse_buttons, keyboard, rotary_knob}`。
- **验证**:拨动拨杆、推动摇杆时,对应接口的值随之变化;遥控器关机后拨杆读数为 UNKNOWN(初值)。

## 阶段 9 — 急停收敛到硬件层

- **需求(问题)**:RMCS 把"任一拨杆 UNKNOWN 或两个都 DOWN 就停"写在 11 个控制器里,新控制器漏写一处就不安全。并且以后会有 3 块板,停止必须对全部板子生效。
- **方法选择 + 原因**:
  - **执行在最底层**:每块板导出一个 GPIO 命令接口 `<board>/enable`。`write()` 时如果 `enable != 1`(包括 NaN),整板所有命令都按 NaN 处理,发 0 电流。
  - **判断集中在一处**:写一个 `SafetyController`,读取拨杆状态接口,按 RMCS 的规则写入所有板的 enable。
  - 默认就是安全的:SafetyController 没有激活或者崩溃时,接口被释放,enable 变回 NaN,所有电机无力。
  - 不选"板子之间通过全局变量通信":那是隐藏通道,别人看不出这层依赖。
- **具体实现**:
  - 底板加 enable 接口;
  - `SafetyController` 的判定规则照抄 `chassis_controller.cpp` 第 106–107 行;
  - 以后每接入一块新板,就在 SafetyController 的接口列表里加一行。
  - SafetyController 同时导出一个状态接口 `safety/enabled`。02 的控制器读它来判断是否失能,并在失能时捕获状态、复位 PID,不再各自判断拨杆。
- **验证**:
  - 两个拨杆都拨到下 → 所有电机无力,此时 PID 控制器仍在运行;
  - 停用 SafetyController → 所有电机无力;
  - 拨回中间位 → 恢复控制。

## 阶段 10 — IMU:底盘 BMI088

- **需求(问题)**:舵轮控制和大 Yaw 的世界系稳定都要用底盘陀螺的 z 轴角速度(`/chassis/yaw/velocity_imu`)。
- **方法选择 + 原因**:
  - `bmi088.hpp` 是纯 std 代码,原样复用,参数和 RMCS 一样用 `Bmi088{1000, 0.2, 0.0}`(Mahony 滤波)。
  - 导出成 ros2_control 的 imu sensor 接口(orientation.x/y/z/w、angular_velocity.x/y/z),这样以后可以直接用 imu_sensor_broadcaster 观察。
- **具体实现**:
  - `accelerometer_receive_callback` / `gyroscope_receive_callback` → `store_*`;
  - `read()` 里调 `update_status()`;
  - 坐标轴映射照抄 `sentry.cpp`。底板不做映射。
- **验证**:原地转动车体,`chassis_imu/angular_velocity.z` 的符号和量级都正确;静止时数值接近 0。

## 阶段 11 — LK 电机:大 Yaw MG6012

- **需求(问题)**:云台的三个电机都是 LK 电机。协议和 DJI 不同:每个电机单独一帧(0x141),角度只在单圈 [0, 2π) 内。
- **方法选择 + 原因**:
  - `lk_motor.hpp` 的协议部分(`generate_*_command` 那一组,第 219–417 行)原样保留,剥离它对 rclcpp 和 executor 接口的依赖。
  - 先只支持力矩命令:RMCS 的云台控制器只输出力矩。角度、速度命令模式等以后有需要再加。
  - NaN 时发 0xA1 零电流帧,这是 RMCS 第 240–269 行已有的行为。这样即使不出力,反馈也会持续回来。
- **具体实现**:
  - 关节 `gimbal_bottom_yaw`(底板 can1,0x141,reversed,减速比 8,零点取自 `calibration.yaml`);
  - `write()` 每拍都发这一帧,和 RMCS 一致。
- **验证**:手转大 Yaw,position 在 [0, 2π) 内连续,过零点时正确回绕;小力矩下转向正确。

## 阶段 12 — 第二块板:顶板 RmcsBoardLite

- **需求(问题)**:pitch、小 Yaw、摩擦轮、拨弹都在顶板上。顶板的 librmcs 类型不同(RmcsBoardLite:can0 到 can3、uart0/1),并且有自己的 USB 线程。
- **方法选择 + 原因**:
  - 每块板一个 SystemInterface,URDF 里写多个 `<ros2_control>` 段,CM 依次调用各板的 read 和 write。
  - "设备表、分发表、帧分组"这部分逻辑抽成两块板共用的基类,只有和 agent 类型相关的回调留在子类里。之所以在这一步才抽,是因为现在第二个使用者出现了。
- **具体实现**:
  - `RmcsBoardLiteSystem`;
  - 关节 `gimbal_pitch`(can2,0x141)、`gimbal_top_yaw`(can3,0x141)、`shooter_friction_r` / `shooter_friction_l` / `shooter_feeder`(can0:0x201 / 0x202 / 0x204,0x200 帧第 1 / 2 / 4 槽;右摩擦轮 reversed;拨弹减速比 38、reversed、multi-turn);
  - 顶板 IMU 的坐标映射是 (-x, -y, z);
  - 顶板也导出 enable 接口,加入 SafetyController 的列表。
- **验证**:顶板单独上电也能通过台架测试;两块板同时在线时,1 kHz 循环没有 overrun 告警。

## 阶段 13 — 第三块板:云台 C 板(只有 IMU)

- **需求(问题)**:云台控制用的是装在 pitch 上的那块 IMU(RMCS 的 `OdomGimbalImu`),它在第三块板上。
- **方法选择 + 原因**:直接复用 `CBoardSystem`,但这块板上没有关节,只有一个 sensor。这一步也正好验证"板级组件可以只有传感器"。
- **具体实现**:
  - 第三个 `<ros2_control>` 段,序列号 `d4-1d2b`;
  - 坐标映射 (y, -x, z);
  - 导出 imu 接口 `gimbal_imu/*`。
- **验证**:转动云台,`gimbal_imu` 的姿态变化和 pitch / yaw 的动作对得上。

## 阶段 14 — 机器人描述与 TF

- **需求(问题)**:
  - 导航、自瞄、可视化这些环外程序都需要 TF。
  - RMCS 的 TF 只存在于进程内部的 fast_tf 里;它的 TfBroadcaster 在 `sentry.yaml` 里还被注释掉了,所以环外完全看不到。
- **方法选择 + 原因**:
  - 用标准 URDF,由 robot_state_publisher 根据 `/joint_states` 发布 TF,这是环外的标准做法。
  - 这一阶段先以底盘作为 `base_link`。`base_link` 的最终语义(底盘,还是雷达所在的大 Yaw 台)由 03 阶段 1 决定;如果那里决定换根,再回来改这里。
  - 环内控制器需要做坐标计算时,继续用 `rmcs_description` / fast_tf(纯头文件,Eigen 实现),这属于内容复用。
  - 两边的几何参数都取自 `sentry_description.hpp`,确保一致。
- **具体实现**:
  - 云台链:`base_link → gimbal_center`(固定)`→ gimbal_bottom_yaw`(绕 Z)`→ gimbal_top_yaw`(绕 Z,偏移 0.08 0 0)`→ gimbal_pitch`(绕 Y)`→ camera`(偏移 0.07128 0 0.0481);
  - 4 个舵轮模块的舵向和轮向关节,位置按 `vehicle_radius` 摆放。RMCS 的轮子 TF 没有平移量,这里要补上;
  - 加 IMU 的 frame。
- **验证**:在 rviz 里看到整车模型,转动各关节时模型跟着动;`ros2 run tf2_tools view_frames` 输出的 TF 树完整。

## 阶段 15 — 板上串口:裁判系统(读方向)

- **需求(问题)**:功率上限、缓冲能量、热量、比赛阶段、血量等数据来自裁判系统串口(底板 uart1)。它是字节流,不是标量。
- **方法选择 + 原因**:
  - 串口归板子所有:回调里把字节推进 `rmcs_utility::RingBuffer`(SPSC 无锁)。
  - 解析放在 `read()` 里。RMCS 的 `referee/status.cpp` 本来就在 1 kHz 线程上解析(CRC8/16),所以线程模型不变。
  - 控制要用的字段逐个导出为状态接口:`referee/{chassis_power_limit, buffer_energy, output_status, shooter_cooling, shooter_heat_limit, game_stage, robot_id, current_hp, bullet_allowance}`。
  - 完整数据交给阶段 17 的广播器发到环外。
  - 写方向(UI、交互)暂时不做,归属还没定。
- **具体实现**:
  - `uart1_receive_callback` → ring buffer;
  - 把 `status.cpp` 第 99–135 行的解析逻辑搬进来;
  - 超时后的回退值照 RMCS(cooling = 40、limit = 50000)。
- **验证**:连上裁判系统后,game_stage 和功率上限的读数正确;拔掉串口线后,回退值生效。

## 阶段 16 — 超电

- **需求(问题)**:
  - 底盘的实测功率来自超电板(can2 0x300)。
  - 超电需要接收"充电功率上限 + 使能"命令(放在 0x1FE 帧第 4 槽)。
  - 在 RMCS 里这两个输入是 required 的,缺任何一个都无法启动。
- **方法选择 + 原因**:
  - 导出两个命令接口:`supercap/charge_power_limit`、`supercap/enable`。它们由 02 的功率控制器写入。没有控制器时两者都是 NaN,按"不充电、不放电"处理,这是安全的默认值。
  - 实测量作为状态接口导出:`chassis/power`、`chassis/voltage`、`supercap/voltage`、`supercap/enabled`。
  - 协议部分取自 `supercap.hpp` 第 52–64 行,删掉注册输入输出的部分。
- **具体实现**:分发 0x300 → `store_status`;`write()` 时把超电命令拼进舵向帧的第 4 槽。
- **验证**:底盘动起来时 `chassis/power` 读数合理;手动写入充电上限后,超电电压上升。

## 阶段 17 — 环内外边界的统一写法

- **需求(问题)**:
  - 导航、决策、自瞄需要读取环内状态:拨杆、裁判数据、云台 yaw、关节状态;
  - 环内又要接收它们发来的指令:底盘速度、云台目标、瞄准方向。
  - 需要一套统一、线程安全、带超时的写法,不能每个控制器各写一套。
- **方法选择 + 原因**:
  - **进环**:控制器订阅话题,用 RealtimeBuffer 保存最新值(单个标量用 atomic 也可以)。消息带 stamp,超时就把参考量置为 NaN。超时时间沿用 RMCS:底盘 500 ms、自瞄 100 ms。
  - **出环**:用广播器。关节状态用 joint_state_broadcaster;非关节接口写一个通用的 `InterfaceBroadcaster`,把参数里列出的状态接口通过 RealtimePublisher 发布出去,作用相当于 RMCS 的 ValueBroadcaster。
  - 不选"控制器里直接 publish":那样会把非实时操作带进 `update()`。
- **具体实现**:
  - 写一个参考量入口的小工具类(subscription + RealtimeBuffer + 超时判断),供 02 和 04 复用;
  - 写 `InterfaceBroadcaster`。
- **验证**:
  - 用 `ros2 topic pub` 发底盘速度,停发 500 ms 后参考量变回 NaN;
  - `ros2 topic echo` 能看到拨杆和裁判数据。

## 阶段 18 — 整车 bringup 模板与 fail fast

- **需求(问题)**:三块板、几十个接口、十几个控制器,启动的顺序和配置必须可重复。配置写错时要在启动期报错,不能等到运行中才发现。
- **方法选择 + 原因**:
  - 只用一个 launch,参数 `robot:=sentry`。配置分两层:
    - `config/sentry/`:车型级配置,包括控制器参数;
    - `calibration.yaml`:单车标定数据。
  - 每条链用一个 spawner 加 `--activate-as-group` 激活;任何一个 spawner 以非零码退出,就通过 `OnProcessExit` 触发 `Shutdown`。
  - 至于 USB 掉线会让进程 terminate 的问题,先沿用 RMCS 的 `respawn=True`(`rmcs.launch.py`),等出现具体问题再考虑改 librmcs。
- **具体实现**:`bringup.launch.py`,外加 `config/sentry/{controllers.yaml, calibration.yaml}`。
- **验证**:
  - 正常启动后所有控制器都是 active;
  - 故意把某个接口名写错,launch 应整体退出,并且日志里指出是哪个接口;
  - 拔掉底板 USB,进程被拉起后能恢复。

---

## 本计划之后

- 硬件和边界都就绪后,进入 `02_control.md`。其中舵轮底盘部分从阶段 6 之后就可以开始,不必等全部完成。
- 工程细节的触发清单见 `architecture.md` §7。每次引入一项,都要写一个与本文同格式的小阶段:现象 → 方法 → 实现 → 验证。

## 输出规范

- 全程中文;代码注释写意图;每个阶段都给出验证命令。
- 每个阶段结束写检查点,内容包括:**新增约束 / 解决的问题 / 代价**。
- 搬运 RMCS 代码时,列出三项:**原样保留的部分 / 删除的耦合 / 修改之处及原因**。
