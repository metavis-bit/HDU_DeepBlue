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
static void NormalizeCommand(char *command);
static uint8_t CommandEquals(const char *expected);
static void thruster_all_stop(void);

// ====================== 运动开启标志 ======================
uint8_t forward_open_flag = 0;        // 直行开启标志
uint8_t rotate_180_open_flag = 0;     // 180度旋转开启标志

// ====================== 目标角度 ======================
float forward_yaw_target = 0.0f;      // 直行目标偏航角
float turn_180_target = 0.0f;         // 180度右转目标角度

// ====================== TURN180 串口闭环追踪 ======================
#define TURN180_TIMEOUT_MS  10000U                // TURN180 超时上限（ms）
static uint8_t  turn180_serial_active = 0;        // 1=由串口 TURN180 触发，需回 DONE/ERR
static uint32_t turn180_start_tick    = 0U;       // TURN180 启动时刻


/* 用户主逻辑函数 */
void UserLogic_Code(void)
{
    printf("Underwater robot starts\r\n");  // 水下机器人启动

    while(1)
    {
        // 清空接收缓冲区
        memset(RaspberryPi_data, 0, sizeof(RaspberryPi_data));

        // 1. 串口DMA接收树莓派指令
        num = Drv_Uart_Receive_DMA(&Uart1, RaspberryPi_data);
        if (num > 0)
        {
            thruster_start_open();  // 解析树莓派指令
        }

        // 2. 执行180度旋转
        if(rotate_180_open_flag == 1)
        {
            jy901_yaw_anti_rotate_open();
        }

        // 2.1 TURN180 串口模式：自然完成 → 回 DONE TURN180
        //     PID 完成时会调用 jy901_yaw_anti_rotation_cancel() 清零 rotate_180_open_flag
        if(turn180_serial_active == 1 && rotate_180_open_flag == 0)
        {
            turn180_serial_active = 0;
            printf("DONE TURN180 %.2f\r\n", JY901S.stcAngle.ConYaw);
        }
        // 2.2 TURN180 串口模式：超时 → 强制水平停止 + 回 ERR TURN180 TIMEOUT
        else if(turn180_serial_active == 1 &&
                (HAL_GetTick() - turn180_start_tick) > TURN180_TIMEOUT_MS)
        {
            jy901_yaw_anti_rotation_cancel();
            turn180_serial_active = 0;
            printf("ERR TURN180 TIMEOUT\r\n");
        }

        // 3. 执行直线行驶（带偏航角稳向）
        if(forward_open_flag == 1)
        {
            jy901_yaw_anti_forward_open();
        }

        // 喂狗（看门狗）
        Drv_IWDG_Feed(&demoIWDG);
    }
}

// 树莓派指令解析函数
void thruster_start_open(void)
{
    NormalizeCommand((char *)RaspberryPi_data);

/*-----------------------------------ASCII 控制命令-----------------------------------*/
    // 串口连通性测试
    if(CommandEquals("PING"))
    {
        printf("ACK PING\r\n");
        return;
    }
    // 全停止：6 个推进器全部归 1500
    if(CommandEquals("STOP") || CommandEquals("ALL_STOP"))
    {
        thruster_all_stop();
        forward_open_flag = 0;
        rotate_180_open_flag = 0;
        turn180_serial_active = 0;
        printf("ACK STOP\r\n");
        return;
    }
    // 仅水平停止
    if(CommandEquals("H_STOP"))
    {
        turn180_serial_active = 0;
        jy901_yaw_anti_rotation_cancel();
        thruster_horizontal_stop();
        printf("ACK H_STOP\r\n");
        return;
    }
    // 仅垂直停止
    if(CommandEquals("V_STOP"))
    {
        thruster_vertical_stop();
        printf("ACK V_STOP\r\n");
        return;
    }
    // 查询当前 yaw（角度，−180~180）
    if(CommandEquals("GET_YAW"))
    {
        printf("YAW %.2f\r\n", JY901S.stcAngle.ConYaw);
        return;
    }
    // 闭环 180 度转向：以当前 yaw 为基准，旋转到 yaw+180
    if(CommandEquals("TURN180"))
    {
        forward_open_flag = 0;
        jy901_yaw_anti_rotation_cancel();   // 重置任何水平动作
        rotate_180_open_flag = 1;           // 启动 PID 闭环 180 度旋转
        turn180_serial_active = 1;          // 标记串口模式，需回 DONE/ERR
        turn180_start_tick = HAL_GetTick();
        printf("ACK TURN180\r\n");
        return;
    }
    // 取消 TURN180：水平停止
    if(CommandEquals("TURN180_CANCEL"))
    {
        turn180_serial_active = 0;
        jy901_yaw_anti_rotation_cancel();
        thruster_horizontal_stop();
        printf("ACK TURN180_CANCEL\r\n");
        return;
    }

/*-----------------------------------垂直方向控制-----------------------------------*/
    // 垂直向上
    if(CommandEquals("JSB 3 Press"))
    {
        thruster_vertical_up();
    }
    // 垂直停止
    else if(CommandEquals("JSB 3 Release") || CommandEquals("JSB 0 Release"))
    {
        thruster_vertical_stop();
    }
    // 垂直向下
    else if(CommandEquals("JSB 0 Press"))
    {
        thruster_vertical_down();
    }

/*-----------------------------------水平方向控制-----------------------------------*/
    // 水平直走（带稳向）
    else if(CommandEquals("JSB 7 Press"))
    {
        turn180_serial_active = 0;
        jy901_yaw_anti_rotation_cancel();  // 先关闭其他稳向功能
        forward_open_flag = 1;             // 开启直行标志
    }
    // 水平后退
    else if(CommandEquals("JSB 4 Press"))
    {
        turn180_serial_active = 0;
        jy901_yaw_anti_rotation_cancel();
        thruster_horizontal_backward();
    }
    // 普通右转（不再触发 180 度闭环；TURN180 改为 ASCII 命令）
    else if(CommandEquals("JSB 5 Press"))
    {
        turn180_serial_active = 0;
        jy901_yaw_anti_rotation_cancel();
        thruster_horizontal_right_turn();
    }
    // 左转
    else if(CommandEquals("JSB 6 Press"))
    {
        turn180_serial_active = 0;
        jy901_yaw_anti_rotation_cancel();
        thruster_horizontal_left_turn();
    }
    // 水平方向各 Release：水平停止
    else if(CommandEquals("JSB 7 Release") ||
            CommandEquals("JSB 4 Release") ||
            CommandEquals("JSB 5 Release") ||
            CommandEquals("JSB 6 Release"))
    {
        turn180_serial_active = 0;
        jy901_yaw_anti_rotation_cancel();
        thruster_horizontal_stop();
    }
    // 全推进器向下测试
    else if(CommandEquals("JSB 11 Press"))
    {
        Drv_PWM_HighLvTimeSet(&thruster[0],1750);
        Drv_PWM_HighLvTimeSet(&thruster[1],1750);
        Drv_PWM_HighLvTimeSet(&thruster[2],1750);
        Drv_PWM_HighLvTimeSet(&thruster[3],1750);
        Drv_PWM_HighLvTimeSet(&thruster[4],1750);
        Drv_PWM_HighLvTimeSet(&thruster[5],1750);
    }
    else if(CommandEquals("JSB 11 Release"))
    {
        thruster_all_stop();
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

    for(i = 0; i < 6; i++)
    {
        Drv_PWM_HighLvTimeSet(&thruster[i], 1500);
    }

    jy901_yaw_anti_rotation_cancel();
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
    int right_pwm = 1700 + right_offset;
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
