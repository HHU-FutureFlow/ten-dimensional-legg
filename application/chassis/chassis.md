# chassis

@Todo 使用条件编译，选择麦轮(全向轮),舵轮,平衡底盘

## 模块结构（2026-11-23 分区整理）

| 文件 | 职责 |
| --- | --- |
| `chassis.c/.h` | 主任务调度骨架：cmd 接收与零力急停沿、挡位斜坡+模型一致 yaw 重构+遥控目标生成、观测组装、台阶开窗协调、LQR 调用、恢复段输出覆盖、腿长力+roll 补偿、电机输出；`ChassisTask()` 为按序调度 |
| `chassis_recovery.c/.h` | 统一恢复状态机 S0~S7（起立/折叠快转/翻身）+ T3 翻倒保护 + 全部 rec 前缀/prot 前缀可调参数；零力沿经 `Recovery_Notify*`、台阶接管经 `Recovery_StepTakeover` 接入 |
| `chassis_step_leg.c/.h` | 接触式上台阶单腿动作状态机（后摆→收腿→回摆→就绪）与 STEP_LEG 容差宏 |
| `chassis_friction.c/.h` | 关节/轮摩擦前馈：候选力矩表、tanh 模型宏、`JointReferenceSpeed`（腿长增量→关节参考速度）、`MotorTorqueWithFriction`（限幅合成） |
| `chassis_utils.c/.h` | `rec_wrap_pi`（角度 wrap，static inline）、`LegVelEst`（500Hz 反馈分隔拍归一 Ldot 估计）、`ChaseReferenceSpeed`（追赶参考速度） |
| `chassis_jump.c/.h` | 跳跃状态机（默认关闭，`WHEELLEG_JUMP_ENABLE`） |
| `grf_observer.c/.h` | GRF 地面反力观测与接地/悬空判定 |
| `step_collision.c/.h` | 台阶碰撞检测（SW1 开窗） |
| `LQR/VMC/balance` | 十维状态反馈、虚拟力到关节力矩映射、腿部运动学观测 |
| `leso_tables.c/.h`、`modules/algorithm/leso.c/.h` | 本地 14 维观测器分带表及更新逻辑，估计左右轮与左右髋的等效扰动力矩 |

各文件内部统一分区顺序：includes → 常量宏 → 可调参数（volatile，ep 在线整定）→ 类型定义 → 文件级状态 → 内部工具 → 对外接口；同区内多功能以横幅注释分节。EP 观测符号名在整理中保持不变。

## 工作流程

首先进行初始化，`ChasissInit()`会被`RobotInit()`调用，进行裁判系统、底盘电机的初始化。如果为双板模式，则还会初始化IMU，并且将消息订阅者和发布者的初始化改为`CANComm`的初始化。

操作系统启动后，工作顺序为：

1. 从cmd模块获取数据（如果双板则从CANComm获取）
2. 判断当前控制数据的模式，如果为停止则停止所有电机
3. 根据控制数据，计算底盘的旋转速度
4. 根据控制数据中yaw电机的编码器值`angle_offset`，将控制数据映射到底盘坐标系下；导航数据也沿用这一套原有转换逻辑
5. 进行麦克纳姆轮的运动学解算，得到每个电机的设定值
6. 获取裁判系统的数据，并根据底盘功率限制对输出进行限幅
7. 由电机的反馈数据和IMU（如果有），计算底盘当前的真实运动速度
8. 设置底盘反馈数据，包括运动速度和裁判系统数据
9. 将反馈数据推送到消息中心（如果双板则通过CANComm发送）


### 后续支持平衡底盘

新增一个app balance_chassis

## 正常平衡与 LESO

2026-11-26 停点基于 KazeYamu 的 9033f77 加本地工作区。用户认可当前平地定点表现，位置积分与松杆制动的进一步整定暂缓。

- `LQR_calc()` 在恢复完成且双轮未受阻时更新 LESO。输入为上一拍的轮与关节力矩指令。左髋两路关节指令求和后取反，还原 VMC 镜像前的虚拟摆矩。估计的扰动力矩乘补偿增益后从 LQR 输出扣除。左髋修正通过普通固件构建和 9 组主机符号检查，工程地图同步至 revision 119。
- 当前本地初值为 `leso_enable=1`、`leso_comp_gain=0.6`、`leso_d_max=6 N·m`。`chassis.c` 的 `leg_theta_offset_rad=-0.07 rad`（约 −4°）在正常平衡时叠加到左右腿参考。EP 读取该参数即可核对偏置，`ten_targets` 中的左右腿目标是叠加前的基础值。
- 修正前两份记录中，1.0 档的直行偏航和腿差大于 0.3 档，松杆位置绝对误差 P95 均约 53 cm。左髋修正后，用户反馈直行偏向消失。`emberprobe-live-20261126-211406.csv` 的 0.6 / 6 N·m 平地近直行样本中，航向误差 P95 约 3.6°，摆角差 P95 约 1.9°，当时代表性停稳静差仍约 38 cm。
- 最新 `emberprobe-live-20261126-215500.csv` 的末行 `s=-0.519 m`、`s_target=-0.512 m`，位置误差约 −6 mm。截图时段 15 次松杆首峰在 0.44–0.51 s 出现。约 0.44 / 0.75 / 1.44 m/s 的松杆实测速度分别对应 8 / 15 / 29 cm 的首次制动位移。`s` 是轮编码器里程。该 CSV 未采集在线偏置、LESO 参数和轮矩。

恢复整定时，固定腿长、负载、偏置与 LESO 参数，用相同实测松杆速度比较首峰、回位时间、末段静差及俯仰。EP 同步采集 `s/ds`、`s_target/ds_target`、两腿摆角、两个 `Tw_cmd` 与 LQR 分带。

## 逐侧触阶试验

SBUS 遥控下，机体完成自起并站稳后将 SW3 置为行驶。SW1 从上拨到下开启一次检测窗口，时长由 `chassis.c` 的 `STEP_COLLISION_WINDOW_MS` 设置，默认 15000 ms。窗口内可松杆，只有正向进给才参与碰撞确认。到时、退出行驶、急停或进入恢复时关窗。

首次实测用支架托住机体。某侧确认触阶后，该轮力矩归零，两台 8009 驱动该腿后摆至机体正后方。保持摆角先收至 0.13 m，实测腿长误差在 0.01 m 内且仍在后摆到位带内时，保持短腿回摆至机体系 −0.17 rad。回摆侧每拍用机体系角 −0.17 ± 0.12 rad 和实测腿长 0.10–0.14 m 判定准备位。先到的一侧保持准备姿态并继续切轮，姿态出范围或反馈无效就退回回摆。两侧同拍到位时清除旧 GRF 离地标记、关闭检测窗并同时交给平衡 LQR，重锚行驶位移和航向目标，下一拍按现有 S5 起立流程恢复腿长。托稳后切入零力可清除动作状态。2026-11-20 实车已记录到双轮恢复与 S5 起立，接管边沿与踏面实际位移待复验。

GRF 继续观测并为碰撞检测筛选样本。离地门控只作用于由正常平衡 LQR 控制的侧，已开始动作的侧由登阶状态机控制。清除旧离地标记后沿用正常检测节拍。姿态到位时的踏面接触位置和接管位移由现场确认。

EP 记录 `step_collision_dbg.hit_mask`、`last_hit_tick`、`step_leg_motion[0/1].phase`、`theta_body_target`、`dbg_len_l/r`、左右实测腿长、`chassis_dbg.theta_body`、`Tw_cmd` 和 `grf_dbg` 的 `F_n_l`、`F_n_r`、`valid_mask`、`air_mask`。碰撞掩码 1 为左、2 为右、3 为双侧，动作相位 0/1/2/3/4 分别为待机、后摆、收短、回摆、准备位。双侧接管时清动作相位，EP 可能同一帧看到两相位回 0、轮控恢复。轮端净空和实际踏面接触位置由现场确认。

## SBUS 跳跃试验

在 robot_def.h 将 WHEELLEG_JUMP_ENABLE 设为 1 后重新构建。默认构建关闭跳跃。
上电、遥控重连或解除急停后，先让左杆竖直通道退出拉杆区，再拉杆蹲伏。
在自起收尾完成后，拉杆使两腿蹲伏。两腿长度和速度连续达标 100 ms 后，
有效松杆持续 100 ms 才转入蹬伸。蹲伏时失去输入授权会取消蹬伸并回站立。
急停清动作状态和回位参考。接触式登阶仍使用 SW1 开窗和每侧碰撞检测。

跳跃相位读取 GRF_Airborne 的双侧离地确认。左右轮仍各用自己的离地位
门控，并各自恢复轮力矩。收腿以原同事相位条件运行；从原快速判据换到主工程
GRF 后的蹬伸到收腿时序、单侧先接地与回位效果均待实车复核。

ChassisJump_Update 生成相位、腿长及 LQR 参考。ChassisJump_LegForce
使用原跳跃阶段的阻尼参考、积分、限幅和前馈。常态试验可在 robot_def.h
分别开启 WHEELLEG_BALANCE_THETA_LEVEL（差模）、WHEELLEG_BALANCE_POSITION_HOLD
（位置保持）、WHEELLEG_BALANCE_LEG_VEL_HOLD（腿速保持和阻尼）。
跳跃差模和腿速保持属于动作控制。EP 可观测 jump_ph、jump_push_abort、
jump_shrink_airok、jump_land_touched、four_control_datas.l_Tleg/r_Tleg 和 grf_dbg.air_mask。
