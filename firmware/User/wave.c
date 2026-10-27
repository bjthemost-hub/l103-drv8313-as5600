#include "wave.h"
#include "debug.h"

/* 采样侧最多就 2 kHz (CS_TASK_MIN_US = 500us), 默认二分频 -> 1 kHz 帧率。
 * 9 B x 1000 = 9 KB/s; 115200 下占约 78% 带宽, 控制台偶尔打几行不影响;
 * 想要 2~4 kHz 就用 'SWB460800' 把口提速。 */
#define WAVE_DIV_DEFAULT    2u
#define WAVE_DIV_MAX        200u

#define WAVE_AMP_SCALE      1000.0f     /* A -> int16, 0.001 A/LSB */

static uint8_t  wv_on   = 0;
static uint16_t wv_div  = WAVE_DIV_DEFAULT;
static uint16_t wv_cnt  = 0;
static uint32_t wv_samp = 0;
static uint32_t wv_frames = 0;

static int16_t amp_to_i16(float a)
{
    float v = a * WAVE_AMP_SCALE;

    if(v >  32767.0f) v =  32767.0f;
    if(v < -32768.0f) v = -32768.0f;
    return (int16_t)v;
}

void Wave_Init(void)
{
    wv_on     = 0;
    wv_div    = WAVE_DIV_DEFAULT;
    wv_cnt    = 0;
    wv_samp   = 0;
    wv_frames = 0;
}

void Wave_SetOn(int on)
{
    wv_on = on ? 1u : 0u;
    wv_cnt = 0;
}

int Wave_IsOn(void)          { return (int)wv_on; }

void Wave_SetDiv(uint16_t div)
{
    if(div < 1u)     div = 1u;
    if(div > WAVE_DIV_MAX) div = WAVE_DIV_MAX;
    wv_div = div;
    wv_cnt = 0;
}

uint16_t Wave_GetDiv(void)   { return wv_div; }
uint32_t Wave_Samples(void)  { return wv_samp; }
uint32_t Wave_Frames(void)   { return wv_frames; }

void Wave_Sample(float iu, float iv, float iw)
{
    uint8_t b[9];

    if(wv_on == 0u) return;

    wv_samp++;

    /* 分频: div=1 每点都发; div=2 每 2 点发一帧。
     * 注意是 "先 ++ 再比", 写成 if(wv_cnt < wv_div) 会变成每 3 点一帧 (2026-10-06 踩过)。 */
    wv_cnt++;
    if(wv_cnt < wv_div) return;
    wv_cnt = 0;

    b[0] = 0xAAu;
    b[1] = 0x55u;
    b[2] = WAVE_T_IMON;

    {
        int16_t su = amp_to_i16(iu);
        int16_t sv = amp_to_i16(iv);
        int16_t sw = amp_to_i16(iw);

        b[3] = (uint8_t)( su       & 0xFF);
        b[4] = (uint8_t)((su >> 8) & 0xFF);
        b[5] = (uint8_t)( sv       & 0xFF);
        b[6] = (uint8_t)((sv >> 8) & 0xFF);
        b[7] = (uint8_t)( sw       & 0xFF);
        b[8] = (uint8_t)((sw >> 8) & 0xFF);
    }

    (void)UsartTx_PutBytes(b, 9);
    wv_frames++;
}

void Wave_PrintInfo(void)
{
    printf("WAVE: %s div=%u (frames ~%lu/s) | samples=%lu frames=%lu txdrop=%lu\r\n",
           wv_on ? "ON " : "OFF",
           (unsigned)wv_div,
           (unsigned long)(2000u / (uint32_t)wv_div),
           (unsigned long)wv_samp, (unsigned long)wv_frames,
           (unsigned long)UsartTx_Drop());
}

int Wave_SetBaud(uint32_t baud)
{
    if(baud < 9600u || baud > 3000000u) return 0;

    Wave_SetOn(0);                 /* 别一边改分频器一边往外发 */
    USART_Printf_Init(baud);       /* 重配 BRR (也会重配 PA9) */
    UsartTx_Init();                /* 清空缓冲 + 重开 TXE 中断 */
    /* 注意: 调用方要**在这之前**把回显打出去 —— 改完再打就是新波特率了, 上位机读到乱码。 */
    return (int)baud;
}
