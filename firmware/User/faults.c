#include "faults.h"
#include "board.h"
#include "debug.h"
#include "can_bus.h"
#include "cmd.h"
#include "csense.h"
#include "drv8313.h"
#include "foc_sensor.h"
#include "foc_utils.h"
#include "foc_motor.h"
#include "motion_pv.h"
#include "pos_log.h"

/* =============================================================================
 * 故障采集 + 主动上报 (说明见 faults.h)
 * 全部跑在主循环里 (Fault_Task), 不在中断里; 发帧走 CAN_Bus_SendStd。
 * ========================================================================== */

#define FAULT_OC_CLEAR_RATIO    0.8f        /* 电流超限回差 (设计值) */
#define FAULT_REJECT_HOLD_US    1000000u    /* "命令被拒"脉冲保持 1s */
#define FAULT_EVT_GAP_US        2000u       /* 两帧事件之间最小间隔 (防刷屏) */
#define FAULT_REPORT_DEF_100MS  10u         /* 默认周期重报 10*100ms = 1s */
#define FAULT_SENSOR_HOLD_US    1000000u    /* 传感器失败后维持报 1s */

static uint16_t active, latched, prev_active;
static uint32_t n_events;
static uint8_t  rep_on, rep_period;
static uint32_t next_period_us, next_evt_us;

static uint8_t  oc_latch, dis_latch, vm_latch, flash_latch, align_latch;
static uint8_t  was_enabled;
static uint32_t flash_fail_seen;
static uint32_t sens_err_seen, sens_err_us;
static uint32_t reject_until_us;
static int16_t  reject_cmd;

static const char *fault_name(uint16_t bit)
{
    switch(bit)
    {
        case CAN_FAULT_STALL:       return "STALL";
        case CAN_FAULT_OVERCURRENT: return "OVERCURRENT";
        case CAN_FAULT_DISABLED:    return "DISABLED";
        case CAN_FAULT_VM_LOW:      return "VM_LOW";
        case CAN_FAULT_SENSOR:      return "SENSOR";
        case CAN_FAULT_NOT_ALIGNED: return "NOT_ALIGNED";
        case CAN_FAULT_FLASH_FAIL:  return "FLASH_FAIL";
        case CAN_FAULT_DRV_NFAULT:  return "DRV_NFAULT";
        case CAN_FAULT_CMD_REJECT:  return "CMD_REJECT";
        default:                    return "?";
    }
}

/* 事件帧 0x1A0:
 *   b0..1 uint16 故障位
 *   b2    0 = 发生 / 1 = 消失
 *   b3    b4..5 是什么 (FAULT_INFO_*)
 *   b4..5 int16  附加数值
 *   b6..7 uint16 事件时刻 [ms, 上电起, 回绕]
 * 返回 1 = 真发出去了, 0 = 被限速挡了 (调用方要把这一位留到下一拍再发, 不能丢) */
static int send_event(uint16_t bit, uint8_t is_clear, uint8_t kind, int16_t info)
{
    uint8_t  d[8];
    uint32_t now = Micros();
    uint16_t ms;

    if((int32_t)(now - next_evt_us) < 0) return 0;
    next_evt_us = now + FAULT_EVT_GAP_US;

    ms = (uint16_t)(now / 1000u);

    d[0] = (uint8_t)(bit & 0xFFu);
    d[1] = (uint8_t)((bit >> 8) & 0xFFu);
    d[2] = is_clear;
    d[3] = kind;
    d[4] = (uint8_t)((uint16_t)info & 0xFFu);
    d[5] = (uint8_t)(((uint16_t)info >> 8) & 0xFFu);
    d[6] = (uint8_t)(ms & 0xFFu);
    d[7] = (uint8_t)((ms >> 8) & 0xFFu);

    if(CAN_Bus_SendStd(CAN_ID_EVT_FAULT, d, 8u) != 0) return 0;

    n_events++;
    return 1;
}

void Fault_SendBitmap(uint16_t id)
{
    uint8_t d[8];

    d[0] = (uint8_t)(active & 0xFFu);
    d[1] = (uint8_t)((active >> 8) & 0xFFu);
    d[2] = (uint8_t)(latched & 0xFFu);
    d[3] = (uint8_t)((latched >> 8) & 0xFFu);
    d[4] = (uint8_t)(n_events & 0xFFu);
    d[5] = (uint8_t)((n_events >> 8) & 0xFFu);
    d[6] = (uint8_t)((n_events >> 16) & 0xFFu);
    d[7] = (uint8_t)((n_events >> 24) & 0xFFu);

    (void)CAN_Bus_SendStd(id, d, 8u);
}

void Fault_Init(void)
{
    active = latched = prev_active = 0u;
    n_events = 0u;
    rep_on = 1u;
    rep_period = FAULT_REPORT_DEF_100MS;
    next_period_us = Micros() + 1000000u;
    next_evt_us = Micros();

    oc_latch = dis_latch = vm_latch = flash_latch = align_latch = 0u;
    was_enabled = (uint8_t)DRV8313_IsEnabled();
    flash_fail_seen = PosLog_WriteFail();
    sens_err_seen = Sensor_ErrorCount();
    sens_err_us = 0u;
    reject_until_us = 0u;
    reject_cmd = 0;
}

/* 外部 (CAN 层) 想报"这条命令被拒了": 给一个 1s 的脉冲 + 记下是哪条 */
void Fault_ReportReject(uint16_t cmd_id, uint16_t reason_bit)
{
    reject_until_us = Micros() + FAULT_REJECT_HOLD_US;
    reject_cmd      = (int16_t)cmd_id;

    if(reason_bit & CAN_FAULT_NOT_ALIGNED) align_latch = 1u;
}

void Fault_Task(void)
{
    uint32_t now = Micros();
    uint16_t want = 0u;
    uint16_t diff;
    uint32_t per;
    uint8_t  i;

    /* ---------------- 1) 采集 ---------------- */
    if(Motion_IsStalled()) want |= CAN_FAULT_STALL;

    /* 电流超限: 窗口峰值 >= 阈值就报, 掉到 80% 以下才撤 */
    {
        float pk = CS_PeakAbs();

        if(pk >= motor_stall_amp)            oc_latch = 1u;
        else if(pk < motor_stall_amp * FAULT_OC_CLEAR_RATIO) oc_latch = 0u;

        if(oc_latch) want |= CAN_FAULT_OVERCURRENT;
    }

    /* 失能: 只认"使能过 -> 没使能"的下降沿 */
    {
        uint8_t en = (uint8_t)DRV8313_IsEnabled();

        if(was_enabled && !en) dis_latch = 1u;
        if(en)                 dis_latch = 0u;
        was_enabled = en;

        if(dis_latch) want |= CAN_FAULT_DISABLED;
    }

    /* 母线电压: 没有采样通道就永远不报 */
    if(CS_VM_Available())
    {
        float v = CS_VM_Volt();

        if(!vm_latch && v < VM_LOW_THRESHOLD) vm_latch = 1u;
        else if(vm_latch && v > VM_LOW_CLEAR) vm_latch = 0u;

        if(vm_latch) want |= CAN_FAULT_VM_LOW;
    }
    else
    {
        vm_latch = 0u;
    }

    /* AS5600: 最近 1s 内有失败就算 */
    if((uint32_t)(Sensor_ErrorCount() - sens_err_seen) != 0u)
    {
        sens_err_seen = Sensor_ErrorCount();
        sens_err_us   = now;
        if(sens_err_us == 0u) sens_err_us = 1u;
    }
    if(sens_err_us != 0u &&
       (uint32_t)(now - sens_err_us) < FAULT_SENSOR_HOLD_US) want |= CAN_FAULT_SENSOR;

    /* Flash 位置保存失败 (写错过一次就锁存, 直到 Fault_Clear) */
    if(PosLog_WriteFail() != flash_fail_seen)
    {
        flash_fail_seen = PosLog_WriteFail();
        flash_latch     = 1u;
    }
    if(flash_latch) want |= CAN_FAULT_FLASH_FAIL;

    /* DRV8313 nFAULT 被拉低 */
    if(DRV8313_Faulted()) want |= CAN_FAULT_DRV_NFAULT;

    /* 未对齐: 只有"因为没对齐被拒过"才报; 对齐成功立刻撤 */
    if(Cmd_IsAligned()) align_latch = 0u;
    if(align_latch)     want |= CAN_FAULT_NOT_ALIGNED;

    /* 命令被拒: 脉冲 */
    if((int32_t)(now - reject_until_us) < 0) want |= CAN_FAULT_CMD_REJECT;

    active = want;

    /* ---------------- 2) 变化 -> 事件帧 ---------------- */
    diff = (uint16_t)(want ^ prev_active);
    if(diff != 0u)
    {
        for(i = 0u; i < 16u; i++)
        {
            uint16_t bit = (uint16_t)(1u << i);
            uint8_t  rise;
            uint8_t  kind = FAULT_INFO_NONE;
            int16_t  info = 0;

            if(!(diff & bit)) continue;

            rise = (want & bit) ? 1u : 0u;
            if(rise) latched |= bit;

            if(bit == CAN_FAULT_CMD_REJECT)
            {
                kind = FAULT_INFO_CMDID;
                info = reject_cmd;
            }
            else if(bit == CAN_FAULT_OVERCURRENT || bit == CAN_FAULT_STALL)
            {
                float a = CS_PeakAmp() * 100.0f;
                kind = FAULT_INFO_AMP;
                info = (int16_t)((a > 32767.0f) ? 32767 : ((a < -32768.0f) ? -32768 : a));
            }
            else if(bit == CAN_FAULT_VM_LOW)
            {
                float v = CS_VM_Volt() * 10.0f;
                kind = FAULT_INFO_VOLT;
                info = (int16_t)((v > 32767.0f) ? 32767 : v);
            }

            /* 限速挡下来的位不更新 prev_active -> 下一拍还会看到这个变化, 不会丢。
             * 总闸 (主动上报) 关着时不发帧, 但下面的状态位照旧维护 —— 被动设备也得记账。 */
            if(rep_on && CAN_Bus_DebugOn() && !send_event(bit, rise ? 0u : 1u, kind, info)) break;

            prev_active = (uint16_t)((prev_active & (uint16_t)~bit) | (want & bit));
        }
    }

    /* ---------------- 3) 周期重报 (位图) ---------------- */
    per = (rep_period != 0u) ? ((uint32_t)rep_period * 100000u) : 100000u;
    if((int32_t)(now - next_period_us) >= 0)
    {
        next_period_us = now + per;
        if(rep_on && CAN_Bus_DebugOn() && rep_period != 0u && want != 0u)
            Fault_SendBitmap(CAN_ID_FB_FAULT);
    }
}

uint16_t Fault_Active(void)     { return active; }
uint16_t Fault_Latched(void)    { return latched; }
uint32_t Fault_EventCount(void) { return n_events; }

void Fault_Clear(uint16_t bits)
{
    latched &= (uint16_t)~bits;

    if(bits & CAN_FAULT_FLASH_FAIL) flash_latch = 0u;
    if(bits & CAN_FAULT_CMD_REJECT) reject_until_us = 0u;
    if(bits & CAN_FAULT_NOT_ALIGNED) align_latch = 0u;
    if(bits & CAN_FAULT_DISABLED)   dis_latch = 0u;
    if(bits & CAN_FAULT_OVERCURRENT) oc_latch = 0u;
    if(bits & CAN_FAULT_VM_LOW)     vm_latch = 0u;
}

void Fault_SetReport(uint8_t on, uint8_t period_100ms)
{
    rep_on     = on ? 1u : 0u;
    rep_period = period_100ms;
}

void Fault_GetReport(uint8_t *on, uint8_t *period_100ms)
{
    if(on)             *on = rep_on;
    if(period_100ms)   *period_100ms = rep_period;
}

void Fault_Print(void)
{
    uint8_t i, on, per;

    Fault_GetReport(&on, &per);

    printf("FAULT: active=0x%04X latched=0x%04X events=%lu\r\n",
           (unsigned)active, (unsigned)latched, (unsigned long)n_events);

    for(i = 0u; i < 16u; i++)
    {
        uint16_t bit = (uint16_t)(1u << i);
        if(active & bit)       printf("FAULT:   [ACTIVE ] %s\r\n", fault_name(bit));
        else if(latched & bit) printf("FAULT:   [latched] %s\r\n", fault_name(bit));
    }

    printf("FAULT: report=%s period=%u (x100ms) | upstream(debug)=%s | VM=%sV sense=%d (thr %sV)\r\n",
           on ? "ON" : "OFF", (unsigned)per, CAN_Bus_DebugOn() ? "ON" : "OFF",
           FmtF(CS_VM_Volt(), 2), CS_VM_Available(), FmtF(VM_LOW_THRESHOLD, 1));
    printf("FAULT: peak=%sA wins=%lu/%u en=%d aligned=%d\r\n",
           FmtF(CS_PeakAmp(), 3), (unsigned long)CS_OverWins(),
           (unsigned)MOTOR_STALL_WINS, Motion_IsEnabled(), Cmd_IsAligned());
}
