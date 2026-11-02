# Plan 03 — 雷达与 SLAM(哨兵:Livox MID-360 · point_lio · 重定位 · 局部地图)

> 用法:把本文件全文发给 AI 助手(编码 Agent)。必须严格按阶段推进:
> - 每个阶段只解决一个问题,改动尽量小;
> - 每个阶段先写"需求 → 方法选择 + 原因",再动手;
> - 阶段结束后停在检查点,等我验证通过再继续。

## 角色

你是我的结对程序员。工作区是 `/home/yuyuyu/Life/项目/RM机器人/mvp_rmcs`。目标是把哨兵的雷达链路(点云 → 里程计 → 建图 → 重定位 → 局部障碍)接进新架构。

- 这些包在 RMCS 里本来就是独立的 ROS2 节点,**以原样复用为主**;
- 我们的工作集中在三件事:接线、坐标系约定、配置,以及补上 RMCS 缺的那几块。

## 背景

内容来源都在 `../RMCS/rmcs_ws/src/` 下:
- `livox_ros_driver2`(MID-360 驱动)
- `point_lio`(改过的 LIO,同时承担里程计和建图采集)
- `rmcs_relocation`(small_gicp VGICP + ScanContext)
- `rmcs-local-map`(局部栅格地图)
- `rmcs-navigation/launch/sensor.launch.yaml`(把以上这些串起来)
- 笔记:`../RMCS/notes/rmcs-local-map-技术笔记.md`

**RMCS 的实际部署状态,需要先认清:**
- 所有决策脚本都用 `global_map = "empty"` 启动(一张空地图),并且没有打开重定位(`api.lua` 第 55 行默认 false;`mini.lua` 第 60–75 行的初始重定位被注释掉了)。
- 所以实车实际在用的只有两样:`world → odom` 是恒等变换,导航只依靠局部地图避障。
- 全局地图和重定位这两块在 RMCS 里**还没打通**,本计划要把它们补齐。

## 铁律

与 01 相同:
- 内容原样复用,只改接线和配置;
- 每个阶段只解决一个问题;
- 工程细节出现问题时才引入;
- 每个阶段都能运行、能验证;
- 每个阶段都按"需求(问题)→ 方法选择 + 原因 → 具体实现 → 验证 → 检查点"的格式写。

---

## 阶段 0 — 驱动跑起来,在 rviz 里看到点云

- **需求(问题)**:先要拿到点云。
- **方法选择 + 原因**:
  - 原样复用 `livox_ros_driver2`。配置文件是 `config/MID360_config.json`:雷达 IP `192.168.100.174`,主机 `192.168.100.1`,端口 56101–56401。这个 IP 在 git 历史里改过,上电前先核对实物。
  - 输出格式保持 CustomMsg(`xfer_format=1`),因为 point_lio 和 local-map 都只吃这个格式。
  - rviz 显示 CustomMsg 不方便,另外用 `xfer_format=0` 起一个 PointCloud2 的实例只看图,或者用驱动自带的 PointCloud2 launch。只为可视化,不进入主链路。
- **具体实现**:
  - 把这几个包作为源码加入工作区(git submodule 或直接复制,都注明来源)。
  - 容器沿用 01 的 `network_mode: host`,宿主机网卡配置静态 IP。
  - 话题名带 IP:`/livox/lidar_192_168_100_174`、`/livox/imu_192_168_100_174`。
- **验证**:
  - rviz 里能看到点云,`ros2 topic hz` 约 10 Hz;
  - IMU 话题有数据。

## 阶段 1 — 安装位置与坐标系约定(最关键的一步)

- **需求(问题)**:
  - point_lio 的位姿发布在 `odom → base_link` 上,而 `base_link` 实际指的是**雷达所在的那个刚体**。
  - RMCS 有两个迹象表明雷达装在大 Yaw 台上,而不是底盘上:
    - ChassisController 收到导航速度后,会按大 Yaw 编码器角把它旋转到底盘系(`chassis_controller.cpp` 第 181–190 行);
    - MotionFsm 用的是 BottomYawLink(`motion.cc` 第 52–85 行)。
  - 雷达的安装参数(z 0.56,倒装 roll 180°)在 `point_lio/config/mid360.yaml` 和 `rmcs-local-map/config/local_map.yaml` 里各写了一份,却没有任何 TF 发布 `livox_frame`。
  - 01 阶段 14 的 URDF 把 `base_link` 定义成了底盘。两边对 `base_link` 的理解冲突,TF 树也不能出现两个父节点。
- **方法选择 + 原因**:
  1. **先看实物**,确认雷达到底装在哪个刚体上。
  2. 如果装在**大 Yaw 台**上(推荐方案):
     - 让 `base_link` 等于大 Yaw 台,也就是 URDF 的根就是雷达所在的刚体。这和 RMCS 导航的语义一致:Nav2 在这个系下规划,不控 wz(MPPI 的 wz_max = 0.001)。
     - 底盘挂在它下面:把 `gimbal_bottom_yaw` 关节**反向定义**,父节点是大 Yaw 台,子节点是底盘,转轴 −Z,`joint_states` 里仍然是电机角。这样 TF 树只有一棵,不需要额外的换算节点。
  3. 如果装在**底盘**上:保持 01 的定义,`base_link` 就是底盘。这时 02 的 A17 不再需要旋转速度。
  4. 雷达的安装位置只在 URDF 里写一次(`base_link → livox_frame` 固定关节)。point_lio 和 local-map 的 yaml 都要和 URDF 保持一致,并在注释里写明以 URDF 为准。
- **具体实现**:
  - 改 01 的 URDF:换根节点,反转关节,加上 `livox_frame`;
  - 更新 02 的 A17 和 04 的阶段 0 中关于坐标系的描述。
- **验证**:
  - `view_frames` 输出一棵完整的树,没有重复的父节点;
  - 转动大 Yaw 时,rviz 里底盘模型相对 `base_link` 反向转动;
  - 雷达点云和车体模型的位置对得上。

## 阶段 2 — point_lio 实时里程计

- **需求(问题)**:要得到一个漂移有界的 `odom → base_link`。导航和建图都依赖它。
- **方法选择 + 原因**:
  - 原样复用 point_lio(RMCS 改过的版本)。它会在估计之前,先把点云和 IMU 转换到 `base_link` 下(`laserMapping.cpp` 第 311–416 行),然后把 LIO 的外参固定为单位阵。
  - `odom` 是重力对齐的,原点在启动时 `base_link` 所在的位置。
  - 配置用 `config/mid360.yaml`,重点核对三项:话题名、`init_pose`(也就是安装外参,和阶段 1 的 URDF 一致)、`acc_norm`。
- **具体实现**:
  - 启动 `pointlio_mapping`;
  - 原 launch 里把 `/tf` 重映射成了 `tf`,保留这个重映射,但要确认命名空间下它确实发到 `/tf`。
- **验证**:
  - 推着车走一圈回到起点,`/odom` 的终点误差在可接受范围内;
  - `/cloud_registered` 在 `odom` 系里是静止的,不会跟着车转。

## 阶段 3 — 录包与回放(离线开发环境)

- **需求(问题)**:后面的建图、重定位、局部地图调参,不能每次都把车开出来。
- **方法选择 + 原因**:
  - 只录原始话题:雷达、IMU,外加 `/joint_states` 和 `/tf_static`。录原始数据,就可以离线重跑任何算法。
  - 回放沿用 RMCS `recall.launch.yaml` 的做法:`ros2 bag play --clock`,所有节点设置 `use_sim_time:=true`。
  - 存储格式用 MCAP。
- **具体实现**:写一个 `record.sh` 和一个 `replay.launch.py`。
- **验证**:回放时 point_lio 的输出和实车运行时一致。

## 阶段 4 — 建图:保存 PCD 先验地图

- **需求(问题)**:重定位需要一张三维先验地图。
- **方法选择 + 原因**:
  - 沿用 RMCS 的做法:point_lio 从启动起的**前 20 分钟**会持续累积点云(`collection.h`),调用 `/save_pcd_map` 服务就会写出 `/tmp/point-lio/<时间戳>.pcd`。不另外写建图节点。
  - 采集方式:慢走、原地转圈、回到起点形成回环,并且在 20 分钟内走完。可以在回放的录包上建图,方便重复。
  - 地图坐标系就是这次建图会话的 `odom`,之后称为 `world`。
- **具体实现**:
  - 写一份采集流程的检查清单;
  - 保存后把 PCD 移到 `mvp_bringup/maps/<场地>/map.pcd`,不能留在 `/tmp`,重启会丢。
- **验证**:用 `pcl_viewer` 打开 PCD,墙面清晰,没有重影。

## 阶段 5 — 二维栅格地图(RMCS 没有,新增)

- **需求(问题)**:Nav2 的全局代价地图需要 `/map`(OccupancyGrid)。RMCS 仓库里的 png 地图没有说明是怎么生成的,也没有从 PCD 生成栅格的工具。
- **方法选择 + 原因**:
  - 写一个离线 Python 脚本:按高度区间截取 PCD(例如离地 0.1–1.0 m 视为障碍),投影到 0.05 m 的栅格上,输出 Nav2 格式的 pgm 和 yaml。
  - 坐标系和 PCD 相同,也就是 `world`。
  - 不选在线的 octomap 之类方案:场地是静态的,离线生成一次就够,简单且可复现。
- **具体实现**:`tools/pcd_to_grid.py`。参数有高度区间、分辨率、膨胀前去噪的点数阈值。
- **验证**:用 map_server 加载后,在 rviz 里和 PCD 叠在一起,墙的位置对得上。

## 阶段 6 — 先验地图上的初始重定位

- **需求(问题)**:每次开机 `odom` 原点都不一样,需要求出 `world → odom`。
- **方法选择 + 原因**:
  - 原样复用 `rmcs_relocation`:small_gicp VGICP,先粗后精两段配准,INITIAL 模式在 ±30° 范围内扫描 yaw。
  - 服务是 `/rmcs_relocation/relocalize`。重定位成功后,它自己以 10 Hz 广播 `world → odom`。
  - 描述子用 `scripts/generate_map_descriptors.py` 离线生成。描述子和地图之间有哈希校验,换了地图必须重新生成。
- **具体实现**:
  - 改 `config/location.yaml` 里的 `map_path` 和 `descriptor_path`,指向阶段 4 保存的地图;
  - 用 `scripts/relocalize_trigger.py`,在 rviz 里点 `/initialpose` 触发重定位。
- **验证**:
  - 在场地内任意放车,点出大致位置后重定位成功;
  - rviz 里实时点云和先验地图重合,`fitness_score` 合理。

## 阶段 7 — 开机自动初始重定位

- **需求(问题)**:比赛时没人会去点 rviz。RMCS 的开机重定位实际上是关着的,导致 `world → odom` 一直是恒等变换,全局地图因此用不上。
- **方法选择 + 原因**:
  - 哨兵的出发位置是已知的(己方出发区)。写一个很小的 `localization_bootstrap` 节点:等到 point_lio 输出稳定后,以出发区位姿作为初值调一次 INITIAL;失败就按间隔重试;把结果发布为状态话题,供 04 的决策读取。
  - 不放进决策 Lua 里:这是"定位就绪"的前置条件,和策略无关,单独成节点更容易测试。
- **具体实现**:
  - 出发区坐标放在场地配置里,红方和蓝方各一份;
  - 节点约 80 行代码;
  - 状态话题 `localization/status`:IDLE / RUNNING / OK / FAILED。
- **验证**:冷启动后无人干预,`world → odom` 能自动建立。

## 阶段 8 — 局部障碍地图

- **需求(问题)**:导航需要近处的实时障碍信息,而且不能依赖全局定位,因为定位丢了也要能避障。
- **方法选择 + 原因**:
  - 原样复用 `rmcs-local-map`:
    - 用陀螺做旋转去畸变;
    - 裁掉盲区 ±0.45 m,只保留 ±8.05 m 范围内的点;
    - 地面不做 RANSAC,而是靠"格内高度差 ≥ 0.1 m 并且至少 5 个点"来隐式剔除;
    - 用射线投射标出自由空间,再做形态学闭运算;
    - 输出 `/local_map`,`base_link` 系,0.04 m 分辨率,402×402。
  - 它只依赖驱动,不依赖 point_lio,所以定位故障时避障仍然有效。
  - 安装参数和阶段 1 的 URDF 对齐。
- **具体实现**:启动这个节点,核对 `local_map.yaml`。里面 `segmentation.*` 等参数并没有实际作用,在注释里标明。
- **验证**:人走到车旁,`/local_map` 上实时出现障碍;地面没有被误判成障碍。

## 阶段 9 — 运行中的漂移修正与被撞恢复

- **需求(问题)**:长时间运行会有漂移;车被撞或被推动后,位置可能整个错掉。
- **方法选择 + 原因**:
  - 复用 `rmcs_relocation` 的 LOCAL 模式:用 ScanContext 选出候选,并有安全门限,单次跳变不超过 0.3 m 或 5°,两次接受之间至少间隔 1 s。修正后 `world → odom` 平滑更新。
  - WIDE 模式用于"彻底迷路",只在停车时调用。
  - 这一阶段只把能力准备好:通过服务就能触发,验证效果。"什么时候触发"由 04 的决策负责(04 阶段 13)。
- **具体实现**:不需要写新代码,只给出调用方式和建议的参数。
- **验证**:
  - 回放一段漂移明显的录包,周期性调用 LOCAL,漂移被拉回;
  - 人为把车搬到别处,停车后调用 WIDE 能找回位置。

---

## 触发清单(出现问题才引入)

| 现象 | 引入什么 | 备注 |
|---|---|---|
| 建图超过 20 分钟走不完 | 改 point_lio 的采集时长参数,或者改成分段建图 | 限制写在 `collection.h` 里 |
| point_lio 中途发散或复位 | 健康监测(速度和协方差突变)加自动重启,然后再做一次 LOCAL 重定位 | — |
| 雷达 IP 冲突、多雷达 | 驱动配置分层 | RMCS 的 IP 改过好几次 |
| 需要 Odin 雷达 | 接入 `odin_ros_driver` | 它自带的 TF(odom → map)会和我们的冲突,接入前要先重新设计 |
| 局部地图把坡道误判成障碍 | 打开 RANSAC 地面分割,或者调高度阈值 | RANSAC 目前被关掉了 |

## 输出规范

- 全程中文;每个阶段都给出验证命令(rviz 配置、`ros2 topic hz`、`view_frames`)。
- 复用的包列出:**原样 / 只改配置 / 改了代码(说明原因)**。
- 每个阶段结束写检查点:**新增约束 / 解决的问题 / 代价**。
