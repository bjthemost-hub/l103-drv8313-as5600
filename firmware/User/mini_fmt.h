#ifndef __MINI_FMT_H
#define __MINI_FMT_H

#include <stdarg.h>

/* =============================================================================
 * 极简 printf / vsnprintf —— 用来替掉 newlib 的 printf 家族
 *
 * 为什么要自己写: 只要调一次 printf 或 vsnprintf, 链接器就会把
 * nano-vfprintf / nano-svfprintf / nano-vfprintf_i / fvwrite / findfp /
 * __sinit / __sfp / makebuf / wsetup / wbuf / fflush / __swbuf_r / __sflush_r /
 * puts / fwalk / reent 这一串全拉进来 (map 里合计约 6 KB Flash),
 * 运行时还要给 stdin/stdout/stderr 各建一个 FILE 和 1 KB 的缓冲 (吃 RAM + 拉进 malloc)。
 * 本工程只需要 %d %s %x 这些, 自己写 200 行就够。
 *
 * 支持:
 *   %d %i %u %x %X %c %s %%
 *   标志: - (左对齐) 0 (补零) + (正号) 空格(正数前留一格)   # 会解析但忽略
 *   宽度: 十进制数字, 如 %04X  %6s  %7lu  % 10u
 *   长度: l  (ll / h / z / j / t 也认, 一律按 32 位处理)
 * 不支持:
 *   浮点 %f/%e/%g —— 这条板子没有 FPU, 工程里一律用 FmtF()/FmtFS() 先转成字符串
 *   精度 .N     —— 会解析但忽略 (现有格式串一个都没用)
 *   %n / 千分位 / 宽字符
 * 用法: 直接写 printf(...) 即可 —— 本头文件把 printf 定义成 mini_printf。
 * ========================================================================== */

int mini_printf(const char *fmt, ...);                                  /* 走 UART, 分块塞环形缓冲 */
int mini_snprintf(char *buf, unsigned int n, const char *fmt, ...);     /* 写进缓冲, 截断(同 snprintf) */
int mini_vsnprintf(char *buf, unsigned int n, const char *fmt, va_list ap);

/* 放在 stdio.h 之后 include 就没问题; 若有人先 include 本头, stdio.h 里
 * printf 的声明会被改写成 mini_printf —— 签名一致, 也无害 */
#ifndef printf
#define printf mini_printf
#endif

#endif
