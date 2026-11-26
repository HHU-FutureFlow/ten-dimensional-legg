// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#if WHEELLEG_REMOTE_HAS_VT02
#include "vt_link.h"
#endif
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

#if WHEELLEG_REMOTE_HAS_RC
static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
#endif
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息
#if WHEELLEG_REMOTE_HAS_RC || WHEELLEG_REMOTE_HAS_VT02
static uint8_t rc_stopped;
static uint8_t mouse_key_stopped;
#endif

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
void RobotCMDInit()
{
    // BMI088_Init_Config_s bmi088_config = {
    //     .cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
    //     .work_mode = BMI088_BLOCK_TRIGGER_MODE,
    //     .spi_acc_config = {
    //         .spi_handle = &hspi1,
    //         .GPIOx = GPIOA,
    //         .cs_pin = GPIO_PIN_4,
    //         .spi_work_mode = SPI_DMA_MODE,
    //     },
    //     .acc_int_config = {
    //         .GPIOx = GPIOC,
    //         .GPIO_Pin = GPIO_PIN_4,
    //         .exti_mode = GPIO_EXTI_MODE_RISING,
    //     },
    //     .spi_gyro_config = {
    //         .spi_handle = &hspi1,
    //         .GPIOx = GPIOB,
    //         .cs_pin = GPIO_PIN_0,
    //         .spi_work_mode = SPI_DMA_MODE,
    //     },
    //     .gyro_int_config = {
    //         .GPIO_Pin = GPIO_PIN_5,
    //         .GPIOx = GPIOC,
    //         .exti_mode = GPIO_EXTI_MODE_RISING,
    //     },
    //     .heat_pwm_config = {
    //         .htim = &htim10,
    //         .channel = TIM_CHANNEL_1,
    //         .period = 1,
    //     },
    //     .heat_pid_config = {
    //         .Kp = 0.5,
    //         .Ki = 0,
    //         .Kd = 0,
    //         .DeadBand = 0.1,
    //         .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //         .IntegralLimit = 100,
    //         .MaxOut = 100,
    //     },
    // };
    //bmi088_test = BMI088Register(&bmi088_config);
#if WHEELLEG_REMOTE_HAS_RC
    rc_data = RemoteControlInit(&huart3);
#endif
#if WHEELLEG_REMOTE_HAS_VT02
    VTLinkInit(&huart1);
#endif
    vision_recv_data = VisionInit(&huart1); // 视觉通信串口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

#if WHEELLEG_REMOTE_HAS_RC
static float vx_cmd_lpf;
static float wz_cmd_lpf;

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *        FS-i6X + iA6B (SBUS),仅实现轮腿的前后移动与机体转向。
 *
 *        语义约定(chassis侧按 chassis_mode 解读):
 *          vx  = 前后速度指令 ds_cmd,单位 m/s,前进为正
 *          vy  = 未使用,恒0
 *          wz  = 转向角速度指令,单位 rad/s,左转为正
 *          chassis_mode = CHASSIS_WHEELLEG_DRIVE(行驶) / CHASSIS_ZERO_FORCE(急停)
 *
 *        注意:本层只产生"操作手意图"(死区+缩放+低通),
 *        s_target/yaw_target 的锚定与积分在 chassis.c 目标生成层完成。
 */
static void RemoteControlSet(const RC_ctrl_t *input)
{
    /* ---- 轮腿行驶参数 ---- */
#define RC_STICK_DEADBAND 20.0f  /* 摇杆死区,±660归一化计数 */
#define WHEELLEG_VX_MAX   1.0f   /* 满杆前进速度 m/s,首次实车勿超1.5 */
#define WHEELLEG_WZ_MAX   2.0f   /* 满杆转向角速度 rad/s,wz>0=左转=LQR yaw正方向 */
#define RC_CMD_LPF_ALPHA  0.15f  /* 指令一阶低通系数(200Hz,τ≈25ms) */

    /* 前后: 右杆竖直(CH2,两种遥控一致);
     * 转向: 左杆水平 —— DT7 是 CH2/rocker_l_,FS-i6X SBUS 是 CH4/rocker_l1(台架确认) */
#if WHEELLEG_REMOTE_MODE == WHEELLEG_REMOTE_DBUS
#define RC_STICK_TURN rocker_l_
#else
#define RC_STICK_TURN rocker_l1
#endif
    float stick_forward = (float)input->rc.rocker_r1;
    float stick_turn    = (float)input->rc.RC_STICK_TURN;

    /* 死区 + 归一化缩放 */
    float vx_cmd = 0.0f, wz_cmd = 0.0f;
    if (stick_forward > RC_STICK_DEADBAND)
        vx_cmd = (stick_forward - RC_STICK_DEADBAND) / (660.0f - RC_STICK_DEADBAND) * WHEELLEG_VX_MAX;
    else if (stick_forward < -RC_STICK_DEADBAND)
        vx_cmd = (stick_forward + RC_STICK_DEADBAND) / (660.0f - RC_STICK_DEADBAND) * WHEELLEG_VX_MAX;
    if (stick_turn > RC_STICK_DEADBAND)
        wz_cmd = -(stick_turn - RC_STICK_DEADBAND) / (660.0f - RC_STICK_DEADBAND) * WHEELLEG_WZ_MAX;
    else if (stick_turn < -RC_STICK_DEADBAND)
        wz_cmd = -(stick_turn + RC_STICK_DEADBAND) / (660.0f - RC_STICK_DEADBAND) * WHEELLEG_WZ_MAX;
    /* 转向取反的原因: 摇杆左推解析为负值(FlySky/DT7 同为右推=+),
     * 而 LQR 约定 yaw 左转为正(LQR.c 列序注释),取反后左推=wz>0=左转。 */

    /* 一阶低通,柔化推杆冲击 */
    vx_cmd_lpf += RC_CMD_LPF_ALPHA * (vx_cmd - vx_cmd_lpf);
    wz_cmd_lpf += RC_CMD_LPF_ALPHA * (wz_cmd - wz_cmd_lpf);

    chassis_cmd_send.vx           = vx_cmd_lpf;
    chassis_cmd_send.wz           = wz_cmd_lpf;
    chassis_cmd_send.chassis_mode = CHASSIS_WHEELLEG_DRIVE;
}

static uint8_t RemoteControlRead(RC_ctrl_t *input)
{
    /* 掉线回调和 UART 解包均会更新遥控状态，在线状态和通道值同取一拍。 */
    const uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    const uint8_t online = RemoteControlIsOnline();
    if (online)
        *input = rc_data[TEMP];
    __set_PRIMASK(irq_state);
    return online;
}

static void EmergencyHandler(const RC_ctrl_t *input, uint8_t online)
{
    if (!online)
        return;

    if (switch_is_down(input->rc.switch_right))
        rc_stopped = 1u;
    else if (switch_is_up(input->rc.switch_right) && rc_stopped)
    {
        /* 用同一物理开关的下→上明确解除，单纯重连在上位不清键鼠急停。 */
        rc_stopped = 0u;
        mouse_key_stopped = 0u;
    }
}

static void RemoteChassisSet(const RC_ctrl_t *input, uint8_t online)
{
    static uint8_t leg_gear_armed;
    const uint8_t ready = online && !rc_stopped && !mouse_key_stopped;
    /* SW2 上电先经过中挡才接管腿长，掉线、急停时撤销许可。 */
    if (!ready)
        leg_gear_armed = 0u;
    else if (switch_is_mid(input->rc.switch_c))
        leg_gear_armed = 1u;

    if (leg_gear_armed)
    {
        if (switch_is_down(input->rc.switch_c))
            chassis_cmd_send.leg_length_target = 0.15f;
        else if (switch_is_up(input->rc.switch_c))
            chassis_cmd_send.leg_length_target = 0.35f;
    }

    const uint8_t drive = ready && switch_is_down(input->rc.switch_left);
    if (drive)
        RemoteControlSet(input);
    else
    {
        /* 遥控器重新接管时，原低通从零使用当前摇杆，防止带回离线前的速度。 */
        vx_cmd_lpf = 0.0f;
        wz_cmd_lpf = 0.0f;
    }

#if WHEELLEG_REMOTE_MODE == WHEELLEG_REMOTE_SBUS
#if WHEELLEG_JUMP_ENABLE
    /* 上电、掉线或急停后先见到在线松杆，防止旧的拉杆跨控制源重放。 */
    static uint8_t jump_input_armed;
    if (!ready)
        jump_input_armed = 0;
    else if (input->rc.rocker_l_ >= -400)
    {
        jump_input_armed = 1;
        chassis_cmd_send.jump_command = CHASSIS_JUMP_RELEASED;
    }
    else if (jump_input_armed)
    {
        chassis_cmd_send.jump_command = CHASSIS_JUMP_HELD;
    }
#endif
    /* 逐侧触阶请求只由本次遥控行驶中的 SW1 上→下触发。 */
    static uint8_t sw1_seen_up;
    const uint8_t sw1 = input->rc.switch_b;
    if (!drive)
        sw1_seen_up = 0u;
    else if (switch_is_up(sw1))
        sw1_seen_up = 1u;
    else if (!switch_is_mid(sw1))
    {
        if (switch_is_down(sw1) && sw1_seen_up)
            ++chassis_cmd_send.step_window_request;
        sw1_seen_up = 0u;
    }
#endif
}
#endif

#if WHEELLEG_REMOTE_HAS_VT02
#define MOUSE_KEY_VX_MAX 0.30f       /* m/s，首轮台架初值，UNTESTED */
#define MOUSE_KEY_WZ_MAX 0.80f       /* rad/s，首轮台架初值，UNTESTED */
#define MOUSE_WZ_PER_COUNT 0.04f
#define MOUSE_SAMPLE_MAX_AGE_MS 40u

typedef struct
{
    uint16_t keys;
    int16_t mouse_x;
    uint8_t online;
    uint8_t resync;
    uint8_t stop_pending;
} MouseKeyInput;

/* 键鼠操作只使用已解码的输入，接收链路与在线仲裁由调用方处理。 */
static void MouseKeySet(MouseKeyInput input, uint8_t selected)
{
    static uint8_t armed;
    static uint8_t neutral_seen;
    static uint16_t previous_keys;
    static float leg_length = 0.25f;

    uint16_t keys = input.keys;

    if (!selected || !input.online || input.resync)
    {
        /* 离开键鼠或链路超时后清许可，下次接管需要中性输入和新的 R 按下沿。 */
        armed = 0u;
        neutral_seen = 0u;
        previous_keys = keys;
    }
    if (!selected)
    {
        leg_length = 0.25f;
        return;
    }

    uint16_t pressed = keys & (uint16_t)~previous_keys;
    previous_keys = keys;

    /* 急停优先，连帧中的 B 即使已释放也必须在本拍生效。 */
    if (input.stop_pending || (input.online && (keys & (1u << Key_B))))
    {
        mouse_key_stopped = 1u;
        armed = 0u;
        neutral_seen = 0u;
        leg_length = 0.25f;
    }
    else if (input.online && !armed && !rc_stopped)
    {
        if (keys == 0u && input.mouse_x == 0)
            neutral_seen = 1u;

        if (neutral_seen && input.mouse_x == 0 &&
            keys == (1u << Key_R) && (pressed & (1u << Key_R)))
        {
            mouse_key_stopped = 0u;
            armed = 1u;
        }
    }
    else if (input.online && armed)
    {
        switch (pressed & ((1u << Key_Z) | (1u << Key_X) | (1u << Key_C)))
        {
        case (1u << Key_Z): leg_length = 0.15f; break;
        case (1u << Key_X): leg_length = 0.25f; break;
        case (1u << Key_C): leg_length = 0.35f; break;
        default: break;
        }
    }

    const uint8_t can_drive = input.online && armed && !rc_stopped && !mouse_key_stopped;
    chassis_cmd_send.leg_length_target = leg_length;

    if (can_drive)
    {
        const int forward = ((keys >> Key_W) & 1) - ((keys >> Key_S) & 1);
        const int turn = ((keys >> Key_A) & 1) - ((keys >> Key_D) & 1);
        chassis_cmd_send.vx = forward * MOUSE_KEY_VX_MAX;
        if (keys & ((1u << Key_A) | (1u << Key_D)))
            chassis_cmd_send.wz = turn * MOUSE_KEY_WZ_MAX;
        else
        {
            /* 鼠标向右为正，底盘左转为正。 */
            float mouse_wz = -input.mouse_x * MOUSE_WZ_PER_COUNT;
            LIMIT_MIN_MAX(mouse_wz, -MOUSE_KEY_WZ_MAX, MOUSE_KEY_WZ_MAX);
            chassis_cmd_send.wz = mouse_wz;
        }

        /* 输入归零时使用普通站立锁定，沿用底盘自己的里程和航向锚定。 */
        if (chassis_cmd_send.vx != 0.0f || chassis_cmd_send.wz != 0.0f)
            chassis_cmd_send.chassis_mode = CHASSIS_WHEELLEG_DRIVE;
    }
}
#endif

static void ChassisControlSet(void)
{
    chassis_cmd_send.jump_command = CHASSIS_JUMP_CANCEL;
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
    chassis_cmd_send.wz = 0.0f;
#if WHEELLEG_REMOTE_MODE == WHEELLEG_REMOTE_NONE
    chassis_cmd_send.chassis_mode = CHASSIS_STANDING_TEST;
#else
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    chassis_cmd_send.leg_length_target = 0.25f;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;

#if WHEELLEG_REMOTE_HAS_RC
    RC_ctrl_t rc_input = {0};
    const uint8_t rc_online = RemoteControlRead(&rc_input);
    EmergencyHandler(&rc_input, rc_online);
    RemoteChassisSet(&rc_input, rc_online);
#elif WHEELLEG_REMOTE_HAS_VT02
    const uint8_t rc_online = 0u;
#endif
#if WHEELLEG_REMOTE_HAS_VT02
    /* VT‑02 只提供键鼠数据。丢失单帧时，过期的鼠标位移按零处理。 */
    static uint32_t vt_session;
    const VTLinkInput vt = VTLinkRead();
    const MouseKeyInput mouse_key = {
        .keys = vt.keys,
        .mouse_x = vt.age_ms < MOUSE_SAMPLE_MAX_AGE_MS ? vt.mouse_x : 0,
        .online = vt.online,
        .resync = vt.session != vt_session,
        .stop_pending = vt.stop_pending,
    };
    vt_session = vt.session;
    /* 遥控器在线时独占底盘指令，包括站立和急停。 */
    MouseKeySet(mouse_key, !rc_online);
#endif

    if (rc_stopped || mouse_key_stopped)
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
#endif
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
   // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    ChassisControlSet();

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // 推送消息,双板通信,视觉通信等
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
