#include "usercode.h"
#include "drv_hal_conf.h"
#include "task_conf.h"
#include "ocd_conf.h"
#include "dev_conf.h"
#include "algo_conf.h"
#include "config.h"
// flow-test: vscode-eide-jlink-github
uint8_t num;
uint8_t RaspberryPi_data[150];  // 树莓派接收数据缓冲区

// ====================== 角度获取标志位 ======================
uint8_t forward_yaw_get_flag = 0;        // 直行时目标偏航角获取标志
uint8_t right_rotate_180_get_flag = 0;   // 180度旋转目标角获取标志

float yaw_current = 0.0f;    // 当前偏航角
int yaw_output = 0;          // PID计算输出值
int left_offset = 0;         // 左侧推进器补偿量
int right_offset = 0;        // 右侧推进器补偿量
uint8_t rotate_180_ready_count = 0;  // 180度旋转到位稳定计数

uint8_t tx_buf[150];         // 发送缓冲区

// 前向声明（定义在文件后部）
static float NormalizeAngle(float angle);
static float AngleDifference(float target, float current);
static float AbsFloat(float value);
static float GetYawDeg(void);
static void NormalizeCommand(char *command);
static uint8_t CommandEquals(const char *expected);
static void UartPollAndParse(void);
static void MotionUpdate(void);
static void thruster_all_stop(void);
static void horizontal_stop(void);
static void vertical_stop(void);
static void start_forward(void);
static void start_backward(void);
static void start_left(void);
static void start_right(void);
static void start_up(void);
static void start_down(void);
static void start_turn180(void);
static void update_turn180(void);
static void forward_yaw_hold_silent(void);
static void turn180_drive_silent(float abs_err);
static void turn_by_pwm_silent(uint8_t turn_right, int pwm_delta);

typedef enum
{
    H_IDLE = 0,
    H_FORWARD,
    H_BACKWARD,
    H_LEFT,
    H_RIGHT,
    H_TURN180
} HorizontalMode;

typedef enum
{
    V_IDLE = 0,
    V_UP,
    V_DOWN
} VerticalMode;

#define H_ACTION_MAX_MS           60000U
#define V_ACTION_MAX_MS           15000U
#define TURN180_TIMEOUT_MS        10000U
#define TURN180_DONE_DEG          8.0f
#define TURN180_SLOW_DEG          45.0f
#define TURN180_DONE_HOLD_MS      350U
#define TURN180_ROTATE_RIGHT      1
#define TURN180_MIN_DELTA_PWM     70
#define TURN180_MAX_DELTA_PWM     260
#define FORWARD_USE_YAW_HOLD      1
#define FORWARD_LEFT_PWM          1720
#define FORWARD_RIGHT_PWM         1720
#define FORWARD_RIGHT_BIAS_PWM    10
#define FORWARD_DEADBAND_DEG      3.0f
#define FORWARD_ADAPT_KP_START    2.0f
#define FORWARD_ADAPT_KP_MIN      0.8f
#define FORWARD_ADAPT_KP_MAX      5.5f
#define FORWARD_MAX_CORR_PWM      100
#define FORWARD_CORR_STEP_PWM     8
#define FORWARD_POS_ERR_TURN_RIGHT 1
#define FORWARD_MIN_PWM           1450
#define FORWARD_MAX_PWM           1850

// ====================== 运动开启标志 ======================
uint8_t forward_open_flag = 0;        // 直行开启标志
uint8_t rotate_180_open_flag = 0;     // 180度旋转开启标志

// ====================== 目标角度 ======================
float forward_yaw_target = 0.0f;      // 直行目标偏航角
float turn_180_target = 0.0f;         // 180度右转目标角度

// ====================== 串口动作状态 ======================
static HorizontalMode h_mode = H_IDLE;
static VerticalMode v_mode = V_IDLE;
static uint32_t h_start_tick = 0U;
static uint32_t v_start_tick = 0U;
static float turn180_start_yaw = 0.0f;
static float turn180_target_yaw = 0.0f;
static uint32_t turn180_done_tick = 0U;


/* 用户主逻辑函数 */
void UserLogic_Code(void)
{
    printf("Underwater robot starts\r\n");  // 水下机器人启动

    while(1)
    {
        UartPollAndParse();
        MotionUpdate();
        Drv_IWDG_Feed(&demoIWDG);
    }
}

// 树莓派指令解析函数
void thruster_start_open(void)
{
    NormalizeCommand((char *)RaspberryPi_data);

/*-----------------------------------ASCII 控制命令-----------------------------------*/
    if(CommandEquals("PING"))
    {
        printf("ACK PING\r\n");
        return;
    }

    if(CommandEquals("STOP") || CommandEquals("ALL_STOP"))
    {
        thruster_all_stop();
        printf("ACK STOP\r\n");
        return;
    }

    if(CommandEquals("H_STOP"))
    {
        horizontal_stop();
        printf("ACK H_STOP\r\n");
        return;
    }

    if(CommandEquals("V_STOP"))
    {
        vertical_stop();
        printf("ACK V_STOP\r\n");
        return;
    }

    if(CommandEquals("GET_YAW"))
    {
        printf("YAW %.2f\r\n", GetYawDeg());
        return;
    }

    if(CommandEquals("TURN180"))
    {
        start_turn180();
        return;
    }

    if(CommandEquals("TURN180_CANCEL"))
    {
        horizontal_stop();
        printf("ACK TURN180_CANCEL\r\n");
        return;
    }

/*-----------------------------------垂直方向控制-----------------------------------*/
    if(CommandEquals("JSB 3 Press"))
    {
        start_up();
        printf("ACK JSB 3 Press\r\n");
    }
    else if(CommandEquals("JSB 3 Release") || CommandEquals("JSB 0 Release"))
    {
        vertical_stop();
        printf("ACK V_RELEASE\r\n");
    }
    else if(CommandEquals("JSB 0 Press"))
    {
        start_down();
        printf("ACK JSB 0 Press\r\n");
    }

/*-----------------------------------水平方向控制-----------------------------------*/
    else if(CommandEquals("JSB 7 Press"))
    {
        start_forward();
        printf("ACK JSB 7 Press\r\n");
    }
    else if(CommandEquals("JSB 4 Press"))
    {
        start_backward();
        printf("ACK JSB 4 Press\r\n");
    }
    else if(CommandEquals("JSB 5 Press"))
    {
        start_right();
        printf("ACK JSB 5 Press\r\n");
    }
    else if(CommandEquals("JSB 6 Press"))
    {
        start_left();
        printf("ACK JSB 6 Press\r\n");
    }
    else if(CommandEquals("JSB 7 Release") ||
            CommandEquals("JSB 4 Release") ||
            CommandEquals("JSB 5 Release") ||
            CommandEquals("JSB 6 Release"))
    {
        horizontal_stop();
        printf("ACK H_RELEASE\r\n");
    }
    else if(CommandEquals("JSB 11 Press"))
    {
        h_mode = H_IDLE;
        v_mode = V_IDLE;
        forward_open_flag = 0;
        rotate_180_open_flag = 0;
        Drv_PWM_HighLvTimeSet(&thruster[0],1750);
        Drv_PWM_HighLvTimeSet(&thruster[1],1750);
        Drv_PWM_HighLvTimeSet(&thruster[2],1750);
        Drv_PWM_HighLvTimeSet(&thruster[3],1750);
        Drv_PWM_HighLvTimeSet(&thruster[4],1750);
        Drv_PWM_HighLvTimeSet(&thruster[5],1750);
        printf("ACK JSB 11 Press\r\n");
    }
    else if(CommandEquals("JSB 11 Release"))
    {
        thruster_all_stop();
        printf("ACK JSB 11 Release\r\n");
    }
    else if(RaspberryPi_data[0] != '\0')
    {
        printf("ERR UNKNOWN_CMD %s\r\n", RaspberryPi_data);
    }
}

// 偏航角稳向直行函数（基础推力1250，PID自动纠偏）
void jy901_yaw_anti_forward_open(void)
{
    // 第一步：首次进入，记录当前偏航角作为直行目标角
    if(forward_yaw_get_flag == 0)
    {
        forward_yaw_target = JY901S.stcAngle.ConYaw;
        forward_yaw_get_flag = 1;
        PID_Clear(&PID);  // 清空PID
    }

    // 第二步：循环进行角度闭环控制
    if(forward_yaw_get_flag == 1)
    {
        yaw_current = JY901S.stcAngle.ConYaw;

        // 最短路径角度差，正确处理 ±180° 跨越
        float angle_diff = AngleDifference(forward_yaw_target, yaw_current);

        // 死区防抖
        if(fabs(angle_diff) < 0.5f)
        {
            forward_adjust(0, 0);
            return;
        }

        // 位置式PID：以归一化角度差为输入，输出绝对纠偏量，避免累积漂移
        yaw_output = PID_Location_Calculate(&PID, 0.0f, angle_diff);

        // 最小有效偏移，避免死区
        if(yaw_output > 0 && yaw_output < 35)
            yaw_output = 35;
        else if(yaw_output < 0 && yaw_output > -35)
            yaw_output = -35;

        left_offset = yaw_output;
        right_offset = -yaw_output;
        forward_adjust(left_offset, right_offset);
    }
}

// ====================== 【核心实现】180度右转闭环控制 ======================
// 角度归一化：限制在 [-180, 180]
static float NormalizeAngle(float angle)
{
    while(angle > 180.0f) angle -= 360.0f;
    while(angle <= -180.0f) angle += 360.0f;
    return angle;
}

// 计算目标角与当前角的最短路径差值
static float AngleDifference(float target, float current)
{
    float diff = target - current;
    while(diff > 180.0f) diff -= 360.0f;
    while(diff <= -180.0f) diff += 360.0f;
    return diff;
}

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float GetYawDeg(void)
{
    return NormalizeAngle((float)JY901S.stcAngle.ConYaw);
}

static void UartPollAndParse(void)
{
    memset(RaspberryPi_data, 0, sizeof(RaspberryPi_data));

    num = Drv_Uart_Receive_DMA(&Uart1, RaspberryPi_data);
    if(num > 0)
    {
        if(num >= sizeof(RaspberryPi_data))
        {
            num = sizeof(RaspberryPi_data) - 1U;
        }

        RaspberryPi_data[num] = '\0';
        thruster_start_open();
    }
}

static void MotionUpdate(void)
{
    uint32_t now = HAL_GetTick();

    if(h_mode == H_FORWARD)
    {
#if FORWARD_USE_YAW_HOLD
        forward_yaw_hold_silent();
#else
        forward_open_loop_silent();
#endif
    }
    else if(h_mode == H_TURN180)
    {
        update_turn180();
    }

    if(h_mode != H_IDLE && h_mode != H_TURN180)
    {
        if((now - h_start_tick) > H_ACTION_MAX_MS)
        {
            horizontal_stop();
            printf("WARN H_TIMEOUT\r\n");
        }
    }

    if(v_mode != V_IDLE)
    {
        if((now - v_start_tick) > V_ACTION_MAX_MS)
        {
            vertical_stop();
            printf("WARN V_TIMEOUT\r\n");
        }
    }
}

static void NormalizeCommand(char *command)
{
    char *start = command;
    size_t len;

    while(*start == ' ' || *start == '\t' || *start == '\r' || *start == '\n')
    {
        start++;
    }

    if(start != command)
    {
        memmove(command, start, strlen(start) + 1);
    }

    len = strlen(command);
    while(len > 0)
    {
        char tail = command[len - 1];
        if(tail != ' ' && tail != '\t' && tail != '\r' && tail != '\n')
        {
            break;
        }
        command[len - 1] = '\0';
        len--;
    }
}

static uint8_t CommandEquals(const char *expected)
{
    return strcmp((char *)RaspberryPi_data, expected) == 0;
}

static void thruster_all_stop(void)
{
    uint8_t i;

    h_mode = H_IDLE;
    v_mode = V_IDLE;
    h_start_tick = 0U;
    v_start_tick = 0U;
    turn180_done_tick = 0U;

    forward_open_flag = 0;
    rotate_180_open_flag = 0;
    forward_yaw_get_flag = 0;
    right_rotate_180_get_flag = 0;
    rotate_180_ready_count = 0;
    PID_Clear(&PID);

    for(i = 0; i < 6; i++)
    {
        Drv_PWM_HighLvTimeSet(&thruster[i], 1500);
    }
}

static void horizontal_stop(void)
{
    h_mode = H_IDLE;
    h_start_tick = 0U;
    turn180_done_tick = 0U;
    rotate_180_ready_count = 0;
    jy901_yaw_anti_rotation_cancel();
}

static void vertical_stop(void)
{
    v_mode = V_IDLE;
    v_start_tick = 0U;
    thruster_vertical_stop();
}

static void start_forward(void)
{
    jy901_yaw_anti_rotation_cancel();
    forward_open_flag = 1;
    rotate_180_open_flag = 0;
    h_mode = H_FORWARD;
    h_start_tick = HAL_GetTick();
#if FORWARD_USE_YAW_HOLD
    forward_yaw_get_flag = 0;
    forward_yaw_hold_silent();
#else
    forward_open_loop_silent();
#endif
}

static void start_backward(void)
{
    jy901_yaw_anti_rotation_cancel();
    forward_open_flag = 0;
    rotate_180_open_flag = 0;
    h_mode = H_BACKWARD;
    h_start_tick = HAL_GetTick();
    thruster_horizontal_backward();
}

static void start_left(void)
{
    jy901_yaw_anti_rotation_cancel();
    forward_open_flag = 0;
    rotate_180_open_flag = 0;
    h_mode = H_LEFT;
    h_start_tick = HAL_GetTick();
    thruster_horizontal_left_turn();
}

static void start_right(void)
{
    jy901_yaw_anti_rotation_cancel();
    forward_open_flag = 0;
    rotate_180_open_flag = 0;
    h_mode = H_RIGHT;
    h_start_tick = HAL_GetTick();
    thruster_horizontal_right_turn();
}

static void start_up(void)
{
    v_mode = V_UP;
    v_start_tick = HAL_GetTick();
    thruster_vertical_up();
}

static void start_down(void)
{
    v_mode = V_DOWN;
    v_start_tick = HAL_GetTick();
    thruster_vertical_down();
}

static void start_turn180(void)
{
    jy901_yaw_anti_rotation_cancel();

    forward_open_flag = 0;
    rotate_180_open_flag = 0;
    turn180_start_yaw = GetYawDeg();
    turn180_target_yaw = NormalizeAngle(turn180_start_yaw + 180.0f);
    turn_180_target = turn180_target_yaw;
    turn180_done_tick = 0U;
    rotate_180_ready_count = 0;
    PID_Clear(&PID);

    h_mode = H_TURN180;
    h_start_tick = HAL_GetTick();

    turn180_drive_silent(180.0f);
    printf("ACK TURN180\r\n");
}

static void update_turn180(void)
{
    float current_yaw;
    float err;
    float abs_err;
    uint32_t now;

    if(h_mode != H_TURN180)
    {
        return;
    }

    now = HAL_GetTick();
    if((now - h_start_tick) > TURN180_TIMEOUT_MS)
    {
        horizontal_stop();
        printf("ERR TURN180 TIMEOUT\r\n");
        return;
    }

    current_yaw = GetYawDeg();
    err = AngleDifference(turn180_target_yaw, current_yaw);
    abs_err = AbsFloat(err);

    if(abs_err <= TURN180_DONE_DEG)
    {
        Drv_PWM_HighLvTimeSet(&thruster[0], 1500);
        Drv_PWM_HighLvTimeSet(&thruster[1], 1500);

        if(turn180_done_tick == 0U)
        {
            turn180_done_tick = now;
        }

        if((now - turn180_done_tick) >= TURN180_DONE_HOLD_MS)
        {
            horizontal_stop();
            printf("DONE TURN180 %.2f\r\n", current_yaw);
            return;
        }
    }
    else
    {
        turn180_done_tick = 0U;
    }

    turn180_drive_silent(abs_err);
}

static void forward_yaw_hold_silent(void)
{
    float current_yaw;
    float error;
    float abs_err;
    float derivative;
    float p_term;
    float i_term;
    float d_term;
    static float integral = 0.0f;
    static float last_error = 0.0f;
    static int last_output = 0;
    static uint8_t large_err_latch = 0U;
    int output;
    int out_max;
    int delta;

    const float DEAD_ZONE = 0.8f;
    const float INT_SEPARATION = 12.0f;
    const float LARGE_ERROR = 25.0f;
    const float INT_MAX = 250.0f;
    const int OUT_MAX_SMALL = 120;
    const int OUT_MAX_LARGE = 220;
    const int MIN_EFFECTIVE = 30;
    const int SLEW_RATE = 18;

    if(forward_yaw_get_flag == 0)
    {
        forward_yaw_target = GetYawDeg();
        forward_yaw_get_flag = 1;
        integral = 0.0f;
        last_error = 0.0f;
        last_output = 0;
        large_err_latch = 0U;
        PID_Clear(&PID);
    }

    current_yaw = GetYawDeg();
    error = -AngleDifference(forward_yaw_target, current_yaw);
    abs_err = AbsFloat(error);

    if(abs_err < DEAD_ZONE)
    {
        integral *= 0.95f;
        last_error = error;

        if(last_output > SLEW_RATE)
            last_output -= SLEW_RATE;
        else if(last_output < -SLEW_RATE)
            last_output += SLEW_RATE;
        else
            last_output = 0;

        left_offset = last_output;
        right_offset = -last_output;
        forward_adjust(left_offset, right_offset);
        return;
    }

    if(abs_err > LARGE_ERROR)
    {
        if(!large_err_latch)
        {
            integral = 0.0f;
            last_error = error;
            large_err_latch = 1U;
        }
    }
    else
    {
        large_err_latch = 0U;
    }

    if(abs_err < INT_SEPARATION)
    {
        integral += error;
        if(integral > INT_MAX) integral = INT_MAX;
        if(integral < -INT_MAX) integral = -INT_MAX;
    }
    else
    {
        integral *= 0.85f;
    }

    derivative = error - last_error;
    last_error = error;

    p_term = PID.fKp * error;
    i_term = PID.fKi * integral;
    d_term = PID.fKd * derivative;

    output = (int)(p_term + i_term + d_term);

    out_max = (abs_err > LARGE_ERROR) ? OUT_MAX_LARGE : OUT_MAX_SMALL;
    if(output > out_max) output = out_max;
    if(output < -out_max) output = -out_max;

    delta = output - last_output;
    if(delta > SLEW_RATE) output = last_output + SLEW_RATE;
    if(delta < -SLEW_RATE) output = last_output - SLEW_RATE;
    last_output = output;

    if(output > 0 && output < MIN_EFFECTIVE) output = MIN_EFFECTIVE;
    if(output < 0 && output > -MIN_EFFECTIVE) output = -MIN_EFFECTIVE;

    yaw_output = output;
    left_offset = output;
    right_offset = -output;
    forward_adjust(left_offset, right_offset);
}

#if !FORWARD_USE_YAW_HOLD
static void forward_open_loop_silent(void)
{
    Drv_PWM_HighLvTimeSet(&thruster[0], FORWARD_LEFT_PWM);
    Drv_PWM_HighLvTimeSet(&thruster[1], FORWARD_RIGHT_PWM);
}
#endif

static void turn180_drive_silent(float abs_err)
{
    int pwm_delta;

    if(abs_err >= TURN180_SLOW_DEG)
    {
        pwm_delta = TURN180_MAX_DELTA_PWM;
    }
    else
    {
        pwm_delta = TURN180_MIN_DELTA_PWM +
                    (int)((TURN180_MAX_DELTA_PWM - TURN180_MIN_DELTA_PWM) *
                          (abs_err / TURN180_SLOW_DEG));
    }

    if(pwm_delta < TURN180_MIN_DELTA_PWM)
    {
        pwm_delta = TURN180_MIN_DELTA_PWM;
    }
    else if(pwm_delta > TURN180_MAX_DELTA_PWM)
    {
        pwm_delta = TURN180_MAX_DELTA_PWM;
    }

    turn_by_pwm_silent(TURN180_ROTATE_RIGHT ? 1U : 0U, pwm_delta);
}

static void turn_by_pwm_silent(uint8_t turn_right, int pwm_delta)
{
    int left_pwm;
    int right_pwm;

    if(turn_right)
    {
        left_pwm = 1500 + pwm_delta;
        right_pwm = 1500 - pwm_delta;
    }
    else
    {
        left_pwm = 1500 - pwm_delta;
        right_pwm = 1500 + pwm_delta;
    }

    if(left_pwm < 1200) left_pwm = 1200;
    if(left_pwm > 1800) left_pwm = 1800;
    if(right_pwm < 1200) right_pwm = 1200;
    if(right_pwm > 1800) right_pwm = 1800;

    Drv_PWM_HighLvTimeSet(&thruster[0], left_pwm);
    Drv_PWM_HighLvTimeSet(&thruster[1], right_pwm);
}

// 180度旋转主函数
void jy901_yaw_anti_rotate_open(void)
{
    const float ROTATE_180_TARGET_TOLERANCE = 1.0f;   // 目标角度容限：±1度停止
    const float ROTATE_180_SLOW_ANGLE = 15.0f;        // 临近目标15度时减速
    const int ROTATE_180_MIN_OFFSET = 50;             // PWM最小有效偏移（防死区）
    const int ROTATE_180_MAX_OFFSET = 200;            // PWM最大偏移限制，增强转弯力度
    const uint8_t ROTATE_180_STABLE_COUNT = 2;        // 稳定2次即停止

    // 【第一步】初始化：只执行一次，记录旋转180度后的目标角度
    if(right_rotate_180_get_flag == 0)
    {
        float current_yaw = JY901S.stcAngle.ConYaw;
        turn_180_target = NormalizeAngle(current_yaw + 180.0f);
        PID_Clear(&PID);
        right_rotate_180_get_flag = 1;
        rotate_180_ready_count = 0;
        forward_open_flag = 0;  // 确保关闭直行
    }

    // 【第二步】循环执行PID角度闭环
    if(right_rotate_180_get_flag == 1)
    {
        yaw_current = JY901S.stcAngle.ConYaw;
        float angle_diff = AngleDifference(turn_180_target, yaw_current);

        // 已到达目标角度：停止推进器
        if(fabs(angle_diff) <= ROTATE_180_TARGET_TOLERANCE)
        {
            rotate_180_ready_count++;
            rotate_180_adjust(0, 0);
            if(rotate_180_ready_count >= ROTATE_180_STABLE_COUNT)
            {
                jy901_yaw_anti_rotation_cancel();
                return;
            }
            return;
        }

        rotate_180_ready_count = 0;

        // 位置式PID：以归一化角度差为输入，避免 ±180° 跨越时误差突变
        yaw_output = PID_Location_Calculate(&PID, 0.0f, angle_diff);

        // 输出限幅，避免死区与超调
        if(yaw_output > 0 && yaw_output < ROTATE_180_MIN_OFFSET)
            yaw_output = ROTATE_180_MIN_OFFSET;
        else if(yaw_output < 0 && yaw_output > -ROTATE_180_MIN_OFFSET)
            yaw_output = -ROTATE_180_MIN_OFFSET;

        if(yaw_output > ROTATE_180_MAX_OFFSET)
            yaw_output = ROTATE_180_MAX_OFFSET;
        else if(yaw_output < -ROTATE_180_MAX_OFFSET)
            yaw_output = -ROTATE_180_MAX_OFFSET;

        // 临近目标时减速
        if(fabs(angle_diff) < ROTATE_180_SLOW_ANGLE)
        {
            if(yaw_output > ROTATE_180_MIN_OFFSET)
                yaw_output = ROTATE_180_MIN_OFFSET;
            else if(yaw_output < -ROTATE_180_MIN_OFFSET)
                yaw_output = -ROTATE_180_MIN_OFFSET;
        }

        left_offset = (int)yaw_output;
        right_offset = -left_offset;
        rotate_180_adjust(left_offset, right_offset);
    }
}

// 关闭所有稳向功能，停止电机
void jy901_yaw_anti_rotation_cancel(void)
{
    h_mode = H_IDLE;
    h_start_tick = 0U;
    turn180_done_tick = 0U;
    forward_yaw_get_flag = 0;
    right_rotate_180_get_flag = 0;
    PID_Clear(&PID);

    // 左右推进器停止
    Drv_PWM_HighLvTimeSet(&thruster[0], 1500);
    Drv_PWM_HighLvTimeSet(&thruster[1], 1500);

    left_offset = 0;
    right_offset = 0;

    forward_open_flag = 0;
    rotate_180_open_flag = 0;
}

// 直行稳向调整（基础1650推力 + PID纠偏）
void forward_adjust(int left_offset,int right_offset)
{
    int right_pwm = 1700 + right_offset + FORWARD_RIGHT_BIAS_PWM;
    int left_pwm  = 1700 + left_offset;

    // PWM限幅保护
    right_pwm = (right_pwm < 1400) ? 1400 : (right_pwm > 2000 ? 2000 : right_pwm);
    left_pwm  = (left_pwm  < 1400) ? 1400 : (left_pwm  > 2000 ? 2000 : left_pwm);

    Drv_PWM_HighLvTimeSet(&thruster[1], right_pwm);
    Drv_PWM_HighLvTimeSet(&thruster[0], left_pwm);
}

// 180度旋转调整（自旋模式，基础1500）
void rotate_180_adjust(int left_offset,int right_offset)
{
    // 偏移量为0 → 直接停止
    if(left_offset == 0 && right_offset == 0)
    {
        Drv_PWM_HighLvTimeSet(&thruster[0], 1500);
        Drv_PWM_HighLvTimeSet(&thruster[1], 1500);
        return;
    }

    const int pwm_deadzone = 40;  // 电机死区

    // 左电机死区处理
    if(left_offset > 0 && left_offset < pwm_deadzone)
        left_offset = pwm_deadzone;
    else if(left_offset < 0 && left_offset > -pwm_deadzone)
        left_offset = -pwm_deadzone;

    // 右电机死区处理
    if(right_offset > 0 && right_offset < pwm_deadzone)
        right_offset = pwm_deadzone;
    else if(right_offset < 0 && right_offset > -pwm_deadzone)
        right_offset = -pwm_deadzone;

    int right_pwm = 1500 + right_offset;
    int left_pwm  = 1500 + left_offset;

    // PWM限幅，提高旋转最大力度
    right_pwm = (right_pwm < 1300) ? 1300 : (right_pwm > 1700 ? 1700 : right_pwm);
    left_pwm  = (left_pwm  < 1300) ? 1300 : (left_pwm  > 1700 ? 1700 : left_pwm);

    Drv_PWM_HighLvTimeSet(&thruster[0], left_pwm);
    Drv_PWM_HighLvTimeSet(&thruster[1], right_pwm);
}
