# Plan 04 — 导航与决策(哨兵:Nav2 · Lua 决策 · 运动模式)

> 用法:把本文件全文发给 AI 助手(编码 Agent)。必须严格按阶段推进:
> - 每个阶段只解决一个问题,改动尽量小;
> - 每个阶段先写"需求 → 方法选择 + 原因",再写代码;
> - 阶段结束后停在检查点,等我验证通过再继续。

## 角色

你是我的结对程序员。工作区是 `/home/yuyuyu/Life/项目/RM机器人/mvp_rmcs`。目标是把 RMCS 哨兵的导航和决策迁到新架构上:
- **环外**是普通 ROS2 节点(Nav2、决策、运动模式);
- **环内**是 ros2_control,只通过 01 阶段 17 定义的参考量入口和广播器与环外交换数据。

## 背景

内容来源是 `../RMCS/rmcs_ws/src/rmcs-navigation/`:
- **Nav2 配置**:`config/motion.yaml`、自定义行为树 `config/motion.xml`;
- **决策**:sol2 + Lua,位于 `src/lua/`;
- **C++ 外壳**:`src/cxx/`,其中有 MotionFsm、目标发送、重定位客户端。

参考笔记:`../RMCS/复盘-navigation2.md`、`rmcs-navigation/AGENTS.md`(部分路径和 IP 已过时)。

RMCS 现在的结构:`Navigation` 是一个 executor 组件,Lua 决策约每 11 拍执行一次,直接跑在 1 kHz 控制线程上。

**这次迁移要顺手修掉的问题:**
- `wait_for_action_server(1s)` 和 `os.execute`(用来启动 tmux)都跑在 1 kHz 线程上,会阻塞控制循环;
- `latest_cmd_vel` 在 ROS 回调里写、在控制线程里读,没有任何同步;
- 导航结果没有回传给 Lua;
- Lua 判断到达用的是 0.3 m 方框,Nav2 用的是 0.2 m;
- MotionFsm 的 SLOPE 分支清的是 `slope_filter`,实际用的却是 `road_filter`(`motion.cc` 第 172 行和第 176 行);
- 云台目标的坐标系不一致(见 02 的 B10)。

**澄清一点**:yaml 里写的策略名 "fast-push-output / kill-robots" 在代码里并不存在。参数 `decision` 只会被打印出来,真正起作用的是 endpoint 脚本(`main`、`mini`、`test`)。

## 铁律

与 01 相同:
- 架构取现代工程,内容取 RMCS(Nav2 配置、行为树、Lua 决策、MotionFsm 原样迁移);
- 每个阶段只解决一个问题;
- 工程细节出现问题时才引入;
- 每个阶段都能运行、能验证;
- 每个阶段都按"需求(问题)→ 方法选择 + 原因 → 具体实现 → 验证 → 检查点"的格式写。

---

## 阶段 0 — 环内外接口与坐标系约定

- **需求(问题)**:导航和决策要往环内发三类东西:底盘速度、底盘行为、云台朝向。这三类数据在哪个坐标系、用什么消息、超时多久,必须先写成约定。RMCS 在这一点上是隐式的,而且有不一致的地方。
- **方法选择 + 原因**:
  - **底盘速度**:`/cmd_vel`,类型 `geometry_msgs/Twist`,坐标系是 `base_link`(按 03 阶段 1 的结论,大概率是大 Yaw 台)。超时 500 ms。环内 ChassisController 负责旋转到底盘系(02 的 A17)。
  - **底盘行为**:uint8,直接复用 `rmcs_msgs::ChassisMode` 的编码。
  - **云台朝向**:yaw 和 pitch,**用云台 IMU 系表示**(02 的 B10)。世界系到 IMU 系的偏置由环外估计。
  - **从环内出来的状态**:拨杆、裁判数据、云台 IMU 的 yaw,由 01 阶段 17 的 InterfaceBroadcaster 发布。
  - **放行条件**(RMCS 里是右拨杆在 UP 才放行)放在环内,不交给决策。
- **具体实现**:
  - 写一份接口约定文档 `docs/interfaces.md`,列出话题名、类型、坐标系、频率、超时;
  - 需要时再加 `mvp_msgs` 领域消息。
- **验证**:约定文档评审通过,和 02 的 A17、B10 逐项对上。

## 阶段 1 — 用 `cmd_vel` 开车(不跑导航)

- **需求(问题)**:要先确认"环外速度 → 环内底盘"这条通路是通的。
- **方法选择 + 原因**:直接用 02 的 A17,不写新代码。用 `ros2 topic pub` 或 teleop_twist_keyboard 发 `/cmd_vel`。
- **验证**:
  - 车按 `base_link` 下的方向运动;
  - 停发 0.5 s 后车停下;
  - 右拨杆不在 UP 时,指令被忽略。

## 阶段 2 — 无雷达的纯软件导航环境

- **需求(问题)**:Nav2 调试需要一套不依赖实车和雷达的环境。
- **方法选择 + 原因**:
  - 沿用 RMCS `static.launch.yaml` 的思路:
    - 用静态 TF 发布 `world → odom` 和 `odom → base_link`;
    - 全局地图和局部地图都用 map_server 加载一张空图。
  - 后面所有阶段都同时保留这个模式,用 `mode:=static` 切换。
- **具体实现**:`navigation_static.launch.py`。
- **验证**:Nav2 各节点都能启动,TF 树完整。

## 阶段 3 — 最小 Nav2:规划 + 跟踪

- **需求(问题)**:给一个目标点,车能自己走过去。
- **方法选择 + 原因**:原样复用 RMCS 的 Nav2 配置:
  - 规划器:ThetaStar(`allow_unknown`,`w_traversal_cost 4`);
  - 跟踪器:MPPI,全向模型,v 范围 ±2,`wz_max` 设成 0.001。这个极小值是为了避开数值奇异,实际效果是从不控制 `base_link` 的朝向,朝向交给云台;
  - 进度检查:0.08 m / 4 s;到达判定半径 0.2 m;
  - 自定义行为树 `motion.xml`:恢复动作只有清空代价地图,没有 Spin 和 BackUp;
  - 生命周期管理器自动启动。
- **具体实现**:
  - 复制 `config/motion.yaml` 和 `motion.xml`,launch 改写为 Python 版;
  - 不加 velocity_smoother。RMCS 文档里出现的 `/cmd_vel_smoothed` 并没有实际使用。
- **验证**:
  - 在阶段 2 的环境里,用 rviz 的 "2D Goal Pose" 发目标,`/cmd_vel` 有输出;
  - 在实车上,把定位换成 03 的 point_lio 后,车能走到目标点。

## 阶段 4 — 局部代价地图接 `/local_map`

- **需求(问题)**:要能绕开实时出现的障碍。
- **方法选择 + 原因**:
  - 沿用 RMCS(`motion.yaml` 第 136–166 行):local_costmap 的 `global_frame` 就是 `base_link`,16 m 滚动窗口;StaticLayer 订阅 `/local_map`(`use_maximum`,lethal 90);膨胀半径 0.3。
  - 因为局部地图本身就在 `base_link` 下,这里不依赖任何 TF。
  - `/local_map` **只进局部代价地图,不进全局**。
- **具体实现**:只改配置;`/local_map` 由 03 阶段 8 提供。
- **验证**:路径上站一个人,车能绕开。

## 阶段 5 — 全局地图 + 重定位接入

- **需求(问题)**:没有全局地图时,规划器不知道场地里哪里是墙,只能靠局部地图边走边绕。RMCS 实际上就是这样运行的。
- **方法选择 + 原因**:全局代价地图改用 03 阶段 5 生成的 `/map`,坐标系是 `world`;`world → odom` 由 03 阶段 6–7 的重定位提供。
- **具体实现**:map_server 加载场地地图;launch 参数 `map:=<场地>`。
- **验证**:规划出的路径会绕开地图上的墙;重定位成功之前,不允许发送导航目标。这个门限由阶段 7 的目标节点负责检查。

## 阶段 6 — MPPI 与全向底盘调参

- **需求(问题)**:实车上出现靠近目标很慢、在窄处卡住、来回摆动等问题。
- **方法选择 + 原因**:按 `AGENTS.md` 第 217–319 行记录的调参经验逐项排查:各向同性的噪声、footprint ±0.2475、到达容差、进度检查。每次只改一组参数,并记录现象和改动。
- **具体实现**:调参日志 `docs/nav_tuning.md`。
- **验证**:场地里的标准路线能稳定跑完。

## 阶段 7 — 目标 API 节点

- **需求(问题)**:决策需要"发目标、换目标、知道到没到、知道失败了没有",而且不能阻塞任何线程。RMCS 的实现:
  - 等待 action server 的调用阻塞在 1 kHz 线程上;
  - 没有处理 feedback;
  - 结果没有回传给 Lua;
  - 失败之后靠每 2 s 重发一次来隐式重试。
- **方法选择 + 原因**:
  - 写一个独立的 `goal_manager` 节点,对外提供:
    - 输入:订阅目标(带请求 ID);
    - 内部:非阻塞地发送 `NavigateToPose` action,新目标会抢占旧目标;
    - 输出:发布状态话题,包含请求 ID、状态(ACTIVE / SUCCEEDED / ABORTED / CANCELED)、剩余距离。
  - 搬 `navigation.cc` 第 202–221 行的去重逻辑:和当前目标相距不到 1 cm 时不重发。
  - 到达判定统一用 Nav2 的结果,不再在 Lua 里自己判断方框。
  - 重定位尚未成功时,拒绝发送目标。
- **具体实现**:Python 或 C++ 均可,约 150 行。
- **验证**:
  - 连续快速发送多个目标,只执行最后一个;
  - 走不过去的目标会报 ABORTED;
  - 整个过程中没有任何线程被阻塞。

## 阶段 8 — 决策上下文

- **需求(问题)**:决策要读一组上下文数据:比赛阶段、血量、弹量、功率上限、拨杆、机器人位姿。RMCS 的做法是在 executor 里直接读这些端口。
- **方法选择 + 原因**:
  - 环内的数据由 01 阶段 17 的 InterfaceBroadcaster 发布(裁判接口和拨杆);
  - 位姿从 TF 查询 `world → base_link`;
  - 写一个 `context` 适配层,把这些数据组装成 Lua 黑板的字段,字段名和 RMCS 的 `blackboard.lua` 保持一致;
  - 保留 RMCS 的 mock 方式(`/rmcs_navigation/context/mock` 发 YAML 字符串),用来做无车调试。
- **验证**:改变拨杆位置、等裁判数据更新,黑板里的字段随之变化。

## 阶段 9 — 决策运行时节点(Lua)

- **需求(问题)**:决策逻辑不能再跑在控制线程上。同时,RMCS 的 Lua 决策代码要原样复用。
- **方法选择 + 原因**:
  - 新建 `mvp_decision` 节点,用 sol2 内嵌 Lua,在自己的定时器上运行。频率先用 10 Hz(RMCS 的 README 写的是 10 Hz,实际约 91 Hz),不够再提高。
  - 原样复制 Lua 部分:
    - `util/{scheduler, fsm, order, edge, clock}.lua`;
    - `blackboard.lua`、`action.lua`;
    - `endpoint/test.lua`。
  - `api` 的 C++ 绑定接口保持不变,只把实现换成 ROS:
    - `send_target`:发给阶段 7 的 goal_manager;
    - `update_gimbal_direction`:发云台朝向话题;
    - `switch_motion_mode` 和 `update_under_attack`:发给阶段 11 的 motion 节点;
    - `update_enable_control`:发一个话题,环内仍然按右拨杆做最终判定。
  - 先只跑 `test` 这一个 endpoint。
- **具体实现**:`lua_context.cc` 的加载流程(`on_init` / `on_tick` / `on_exit`)搬过来;ROS 参数注入到 Lua 的 `option` 表。
- **验证**:按 test 脚本的拨杆序列(中 → 上 → 中)操作,能切换到 attack 模式;Lua 报错不影响控制循环。

## 阶段 10 — 意图:巡逻、守点、进攻、回补

- **需求(问题)**:要有一个完整的比赛策略。
- **方法选择 + 原因**:
  - 搬 `endpoint/mini.lua` 和 `intent/*`:
    - 等比赛阶段变为 STARTED 后才开始;
    - 每拍调用一次 `decide()`;
    - 规则:不在出发区就回补(ESCAPE);弹量 > 290 进攻;> 250 守点;否则巡逻。
  - 意图都是可取消的任务;`task/navigate-to.lua` 的超时 30 s、停留 5 s 原样保留。到达判定改成读阶段 7 的结果。
  - 场地坐标沿用 `blackboard.lua` 里的常量,红方和蓝方各一套。
- **验证**:在阶段 8 的 mock 上下文里,改血量和弹量,能看到意图正确切换;实车巡逻路线能跑通。

## 阶段 11 — 运动模式节点(MotionFsm)

- **需求(问题)**:不同情形要用不同的底盘行为:
  - 进攻时小陀螺;
  - 被打时加速转;
  - 走大路时要平滑,卡住时要倒车。
- **方法选择 + 原因**:
  - 把 `controller/motion.cc` 的 MotionFsm 搬进环外的 `motion` 节点,放在 Nav2 的 `cmd_vel` 和环内入口之间。
  - 各模式的行为(原样搬):
    - NORMAL:直通;
    - ROAD:速度低通滤波 τ = 1 s,最低速度 0.5;500 ms 内移动不足 0.1 m 时倒车 500 ms;
    - ATTACK:受到攻击时 SPIN_FAST,否则 SPIN_SLOW;
    - STEP 和 SLOPE。
  - 判定受攻击搬 `robot_status.lua`:血量下降后,5 s 内都算受攻击。
  - 修 RMCS 的 SLOPE 滤波器用错的问题。
  - 放在环外的理由:它只需要跟着 `cmd_vel` 的频率(20 Hz)运行,不需要 1 kHz。
- **具体实现**:
  - Nav2 的输出重映射到 `/cmd_vel_nav`;
  - motion 节点输出 `/cmd_vel` 和 `chassis_behavior`。
- **验证**:分别切到各个模式,观察底盘行为;人为把车顶住,会触发倒车。

## 阶段 12 — 云台扫描与指向

- **需求(问题)**:
  - 没有发现目标时,云台要扫描;
  - 有目标点时,云台要看向它;
  - 导航指向和自瞄要有明确的优先级。
- **方法选择 + 原因**:
  - 搬 `action.lua` 第 36–69 行:scanning 是余弦扫描,toward 是保持一个角度。
  - 世界系到云台 IMU 系的偏置:搬 `motion.cc` 第 52–85 行的偏置跟踪(增益 0.05,只在世界 yaw 变化时更新)。**偏置要相对云台 IMU 系来计算**,这样就修掉了 RMCS 拿 `OdomImu` 做比较的不一致。
  - 仲裁:环内 02 的 B10 已经规定了"导航 > 自瞄 > 手动"。RMCS 的 `enable_autoaim` 和 `gimbal_dominator` 两个开关从来没用上,这里不迁移;以后需要"导航期间也允许自瞄接管"时再加,列入触发清单。
- **验证**:扫描时云台平滑往复;发出指向后,云台看向世界系里的那个点。

## 阶段 13 — 决策驱动重定位

- **需求(问题)**:要决定什么时候做 LOCAL 修正、什么时候做 WIDE 找回。03 只提供了能力,决策时机在这里定。
- **方法选择 + 原因**:
  - 搬 `action.lua` 第 163–215 行的 `relocalize_*` 和 `send_and_await`(每 0.1 s 轮询一次状态),C++ 端是一个异步服务客户端(`engine.cc`)。
  - 触发策略先定得简单:
    - 停车并且空闲时,周期性做 LOCAL;
    - 位姿明显异常(例如跑出场地边界)时,停车做 WIDE。
- **验证**:回放漂移明显的录包,决策会触发 LOCAL 并修正位姿。

## 阶段 14 — 进程管理:取代 tmux 和 `os.execute`

- **需求(问题)**:RMCS 用 Lua 调 `os.execute` 在 tmux 里拉起整个导航栈(`api.lua` 第 43–87 行)。这样会阻塞调用线程,容易留下僵尸进程,而且不可观测。
- **方法选择 + 原因**:
  - 所有环外节点都由 bringup launch 统一启动;
  - Nav2 节点交给自己的 lifecycle manager 管理;
  - 决策里原来的"重启导航",改成调用 lifecycle manager 的 reset 和 startup 服务;
  - 进程级的崩溃用 launch 的 `respawn` 处理。
- **验证**:决策触发重启导航时,控制循环不受影响,也不会留下残余进程。

---

## 触发清单(出现问题才引入)

| 现象 | 引入什么 | RMCS 状态 |
|---|---|---|
| 卡在 U 型陷阱里 | 无进展时生成局部子目标脱困 | `AGENTS.md` 第 321–332 行有设计,没有实现 |
| 需要不上车调策略 | 决策离线仿真(Godot sidecar,`src/sim`) | API 已经和代码不同步(缺少 `switch_motion_mode` 等),要先修 |
| 导航期间也想让自瞄接管云台 | `gimbal_dominator` 仲裁 | 字段存在,但从来没有生效 |
| 需要给裁判系统发哨兵决策指令 | 裁判"写"方向的归属(`architecture.md` §9) | `sentry_decision.cpp` 是空文件 |
| 决策需要更高频率 | 提高定时器频率 | RMCS 实际约 91 Hz |
| Lua 的 `update_gimbal_direction(nil, nil)` 行为不明 | 在绑定层把 nil 显式转换成 NaN | `mini.lua` 从不设置云台模式 |

## 输出规范

- 全程中文;每个阶段都给出验证命令(rviz、`ros2 action`、`ros2 topic echo`)。
- 复用的内容列出:**原样 / 只改接线 / 改了代码(说明原因)**。
- 每个阶段结束写检查点:**新增约束 / 解决的问题 / 代价**。
