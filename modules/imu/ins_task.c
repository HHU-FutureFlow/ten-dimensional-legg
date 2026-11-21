/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @author  annotation and modificaiton by neozng
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "ins_task.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "spi.h"
#include "tim.h"
#include "user_lib.h"
#include "general_def.h"
#include "master_process.h"

static INS_t INS;
static IMU_Param_t IMU_Param;
static PIDInstance TempCtrl = {0};

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

// 用于获取两次采样之间的时间间隔
static uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
static float RefTemp = 40; // 恒温设定温度

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);

/* ===== 陀螺 z 轴零偏在线校准（2026-11-12）=====
 * 为什么需要:
 *   QuaternionEKF.c:198 把 GyroBias[2] 硬置 0（注释: "z轴通天,无法观测yaw的漂移"），
 *   ins_task.c:101 use_mag=0（无磁力计）。
 *   → yaw 是纯陀螺积分，芯片零偏每帧累加，实测抖擞非常明显。
 *
 * 做法:
 *   上电后前 N 帧累加 z 轴角速度求均值当作零偏，之后每帧减掉。
 *   车必须静止（默认静默期 15s 完全覆盖校准窗口）。
 *   只动 z 轴；x/y 轴仍由 EKF 自行估计（它们有重力观测，本来就不漂）。
 *
 * 为什么能省事:
 *   零偏是常量项，积分后无限累积；扣掉它就只剩随机噪声，
 *   随机噪声积分是随机游走（√t 增长），比线性漂移慢一个数量级。
 *
 * 调参: gyro_calib_enable=0 可整体关闭；
 *       gyro_bias_z_dbg 可直接用 EmberProbe 看校准结果
 *       （典型 BMI088 零偏量级 ±0.01 rad/s，超过 0.05 就要怀疑芯片/焊接）。
 *
 * 【2026-11-14 重做】旧做法“跳过 3s 后无条件采 2s 求均值”有致命缺陷:
 *   若这 2s 内车在动(自起/PUSH 快转), 算出来的就是“零偏 + 平均转动”。
 *   之后每帧都把它扣掉 -> 车真以 4.66°/s 转时读数正好为 0,
 *   yaw 冻住, 控制器以为没转, 于是越转越稳。现场表现: 一直原地打转,
 *   而日志里 dyaw 均值≈ 0、yaw 60s 只变 1.5°。
 * 新做法: 200ms 滑窗 + 双门限判静止, 只有窗口内
 *           |均值| < GYRO_CALIB_MEAN_MAX  且  峰峰值 < GYRO_CALIB_PP_MAX
 *         才把窗均值锁成零偏; 任何一帧不满足就丢窗重来。
 *         关键: 只靠峰峰值不够 —— 匀速旋转的峰峰值也是 0,
 *               所以必须同时看均值(匀速转动会被均值门限拦住)。
 *         超时(30s 都拿不到干净窗)则 bias 保持 0 —— 宁可让它诚实漂,
 *         也不能冻住 yaw(冻住比漂更危险, 因为控制器永远看不到转动)。
 * gyro_calib_done_dbg: 1 = 正常锁定, 2 = 超时兜底(bias=0) */
#define GYRO_CALIB_SKIP      3000    /* 跳过前 3s: 等 BMI088 上电稳定 */
#define GYRO_CALIB_WIN        200    /* 静窗长度 200 拍 = 200ms @1kHz */
#define GYRO_CALIB_PP_MAX     0.04f  /* 窗内峰峰值上限 rad/s (0.04 = 2.3°/s) */
#define GYRO_CALIB_MEAN_MAX   0.03f  /* 窗均值绝对值上限 rad/s (0.03 = 1.7°/s) */
#define GYRO_CALIB_TIMEOUT   30000   /* 30s 拿不到干净窗就兑底 */
volatile uint16_t gyro_calib_frames = 2000;   /* 已废弃(旧固定窗口长度), 保留变量避免旧 Watch 报错 */
volatile uint8_t  gyro_calib_enable = 1;      /* 0 = 关闭校准 */
volatile float    gyro_bias_z_dbg   = 0.0f;   /* 校准结果，供 EmberProbe 观测 */
volatile uint8_t  gyro_calib_done_dbg = 0;    /* 1 = 已锁定, 2 = 超时兑底 */
volatile float    gyro_calib_mean_dbg = 0.0f; /* 最后一窗的均值, 用于在线观察 */
volatile float    gyro_calib_pp_dbg   = 0.0f; /* 最后一窗的峰峰值 */
volatile uint16_t gyro_calib_tries_dbg = 0;   /* 已丢弃的窗数 */
static float    gyro_bias_z    = 0.0f;
static float    gyro_win_sum   = 0.0f;
static float    gyro_win_min   = 0.0f;
static float    gyro_win_max   = 0.0f;
static uint16_t gyro_win_cnt   = 0;
static uint16_t gyro_calib_cnt = 0;
static uint32_t gyro_calib_timeout = 0;
static uint8_t  gyro_calib_done = 0;

static void IMUPWMSet(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}

/**
 * @brief 温度控制
 *
 */
static void IMU_Temperature_Ctrl(void)
{
    PIDCalculate(&TempCtrl, BMI088.Temperature, RefTemp);
    IMUPWMSet(float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
static void InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
    float axis_rot[3] = {0};           // 旋转轴
    // 读取100次加速度计数据,取平均值作为初始值
    for (uint8_t i = 0; i < 100; ++i)
    {
        BMI088_Read(&BMI088);
        acc_init[X] += BMI088.Accel[X];
        acc_init[Y] += BMI088.Accel[Y];
        acc_init[Z] += BMI088.Accel[Z];
        DWT_Delay(0.001);
    }
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;
    Norm3d(acc_init);
    // 计算原始加速度矢量和导航系重力加速度矢量的夹角
    float angle = acosf(Dot3d(acc_init, gravity_norm));
    Cross3d(acc_init, gravity_norm, axis_rot);
    Norm3d(axis_rot);
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // 轴角公式,第三轴为0(没有z轴分量)
}

attitude_t *INS_Init(void)
{
    if (!INS.init)
        INS.init = 1;
    else
        return (attitude_t *)&INS.Gyro;

    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

    while (BMI088Init(&hspi1, 1) != BMI088_NO_ERROR)
        ;
    IMU_Param.scale[X] = 1;
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);
    // imu heat init
    PID_Init_Config_s config = {.MaxOut = 2000,
                                .IntegralLimit = 300,
                                .DeadBand = 0,
                                .Kp = 1000,
                                .Ki = 20,
                                .Kd = 0,
                                .Improve = 0x01}; // enable integratiaon limit
    PIDInit(&TempCtrl, &config);

    // noise of accel is relatively big and of high freq,thus lpf is used
    INS.AccelLPF = 0.0085;
    DWT_GetDeltaT(&INS_DWT_Count);
    return (attitude_t *)&INS.Gyro; // @todo: 这里偷懒了,不要这样做! 修改INT_t结构体可能会导致异常,待修复.
}

/* 注意以1kHz的频率运行此任务 */
void INS_Task(void)
{
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};

    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    // ins update
    if ((count % 1) == 0)
    {
        BMI088_Read(&BMI088);

        INS.Accel[X] = BMI088.Accel[X];
        INS.Accel[Y] = BMI088.Accel[Y];
        INS.Accel[Z] = BMI088.Accel[Z];
        INS.Gyro[X] = BMI088.Gyro[X];
        INS.Gyro[Y] = BMI088.Gyro[Y];
        INS.Gyro[Z] = BMI088.Gyro[Z];

        // demo function,用于修正安装误差,可以不管,本demo暂时没用
        IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

        /* ---- 陀螺 z 轴零偏校准（2026-11-14 重做为静止窗口检测）----
         * 必须在 IMU_QuaternionEKF_Update 之前执行。详见文件头部注释。 */
        if (!gyro_calib_done)
        {
            if (!gyro_calib_enable)
            {
                gyro_calib_done     = 1;
                gyro_calib_done_dbg = 1;         /* 手动关闭, 不抹零偏 */
            }
            else if (gyro_calib_cnt < GYRO_CALIB_SKIP)
            {
                gyro_calib_cnt++;                /* 热机期, 先不采 */
            }
            else
            {
                float gz = INS.Gyro[Z];

                if (gyro_win_cnt == 0)
                {
                    gyro_win_min = gz;
                    gyro_win_max = gz;
                }
                if (gz < gyro_win_min) gyro_win_min = gz;
                if (gz > gyro_win_max) gyro_win_max = gz;
                gyro_win_sum += gz;
                gyro_win_cnt++;
                gyro_calib_timeout++;

                if (gyro_win_cnt >= GYRO_CALIB_WIN)
                {
                    float mean = gyro_win_sum / (float)GYRO_CALIB_WIN;
                    float pp   = gyro_win_max - gyro_win_min;
                    gyro_calib_mean_dbg = mean;
                    gyro_calib_pp_dbg   = pp;

                    if (fabsf(mean) < GYRO_CALIB_MEAN_MAX && pp < GYRO_CALIB_PP_MAX)
                    {
                        /* 真静止: 锁定零偏 */
                        gyro_bias_z         = mean;
                        gyro_bias_z_dbg     = gyro_bias_z;
                        gyro_calib_done     = 1;
                        gyro_calib_done_dbg = 1;
                    }
                    else
                    {
                        /* 车在动(或匀速旋转): 这个窗作废, 重头再来 */
                        gyro_calib_tries_dbg++;
                        gyro_win_cnt = 0;
                        gyro_win_sum = 0.0f;
                    }
                }

                if (!gyro_calib_done && gyro_calib_timeout > GYRO_CALIB_TIMEOUT)
                {
                    /* 30s 都没静止过: bias 保持 0。
                     * 宁可让 yaw 诚实滑, 也不能冻住 —— 冻住会让控制器完全失明。 */
                    gyro_bias_z         = 0.0f;
                    gyro_bias_z_dbg     = 0.0f;
                    gyro_calib_done     = 1;
                    gyro_calib_done_dbg = 2;
                }
            }
        }
        else
        {
            INS.Gyro[Z] -= gyro_bias_z;             /* 每帧扣掉零偏 */
        }

        // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
        // INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
        // INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

        // 核心函数,EKF更新四元数
        IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; ++i) // 同样过一个低通滤波
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

        INS.Yaw = QEKF_INS.Yaw;
        INS.Pitch = QEKF_INS.Pitch;
        INS.Roll = QEKF_INS.Roll;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;

        VisionSetAltitude(INS.Yaw, INS.Pitch, INS.Roll);
    }

    // temperature control
    if ((count % 2) == 0)
    {
        // 500hz
        IMU_Temperature_Ctrl();
    }

    if ((count++ % 1000) == 0)
    {
        // 1Hz 可以加入monitor函数,检查IMU是否正常运行/离线
    }
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
