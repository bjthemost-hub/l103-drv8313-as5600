#include "mini_fmt.h"

/* debug.c 里实现: 往 USART1 的 2048 B 环形缓冲塞, 由 TXE 中断真正发出去 */
extern int _write(int fd, char *buf, int size);

#define MINI_CHUNK  64u         /* 每凑够这么多字节往 UART 塞一次。
                                 * 别调太大: MiniOut 是栈上变量, 而 cprintf 自己已经占了
                                 * 384 B, 本工程栈只有 2048 B (Link.ld __stack_size) */

typedef struct
{
    char         *buf;          /* stream=0 时的目标缓冲 (可为 NULL, 那就只计数) */
    unsigned int  cap;          /* buf 容量(含结尾 0) */
    unsigned int  n;            /* 已经产出的字节数 (同 snprintf 的返回值) */
    int           stream;       /* 1 = 走 _write; 0 = 写 buf */
    char          chunk[MINI_CHUNK];
    unsigned int  cn;
} MiniOut;

static void mini_flush(MiniOut *o)
{
    if(o->cn)
    {
        _write(1, o->chunk, (int)o->cn);
        o->cn = 0;
    }
}

static void mini_put(MiniOut *o, char c)
{
    if(!o->stream)                                  /* 缓冲模式: 超了只计数不写 */
    {
        if(o->buf && (o->n + 1u) < o->cap) o->buf[o->n] = c;
        o->n++;
        return;
    }
    if(o->cn >= MINI_CHUNK) mini_flush(o);
    o->chunk[o->cn++] = c;
    o->n++;
}

static void mini_put_s(MiniOut *o, const char *s)
{
    if(!s) s = "(null)";
    while(*s) mini_put(o, *s++);
}

/* 整数输出: v 已经是无符号量, sign 是符号字符(0 = 没有) */
static void mini_uint(MiniOut *o, unsigned long v, unsigned long base, int upper,
                      int width, int zero, int left, char sign)
{
    char tmp[11];                                   /* 32 位二进制也才 32 位, 10 进制最多 10 位 */
    int  k = 0, pad;

    if(v == 0ul) tmp[k++] = '0';
    while(v)
    {
        unsigned long d = v % base;
        tmp[k++] = (char)(d < 10ul ? ('0' + (char)d)
                                   : ((upper ? 'A' : 'a') + (char)(d - 10ul)));
        v /= base;
    }

    pad = width - k - (sign ? 1 : 0);
    if(!left && !zero) { while(pad-- > 0) mini_put(o, ' '); }
    if(sign)           { mini_put(o, sign); }
    if(!left && zero)  { while(pad-- > 0) mini_put(o, '0'); }
    while(k > 0)       { mini_put(o, tmp[--k]); }
    if(left)           { while(pad-- > 0) mini_put(o, ' '); }
}

static void mini_vformat(MiniOut *o, const char *fmt, va_list ap)
{
    char c;

    while((c = *fmt++) != 0)
    {
        int  left, zero, plus, space, width, islong;
        char sign;

        if(c != '%') { mini_put(o, c); continue; }

        left = zero = plus = space = width = islong = 0;

        c = *fmt++;
        for(;;)                                     /* 标志 */
        {
            if(c == '-') { left  = 1; c = *fmt++; continue; }
            if(c == '0') { zero  = 1; c = *fmt++; continue; }
            if(c == '+') { plus  = 1; c = *fmt++; continue; }
            if(c == ' ') { space = 1; c = *fmt++; continue; }
            if(c == '#') {            c = *fmt++; continue; }   /* 解析但忽略 */
            break;
        }
        while(c >= '0' && c <= '9') { width = width * 10 + (c - '0'); c = *fmt++; }
        if(c == '.')                                /* 精度: 解析但忽略 */
        {
            c = *fmt++;
            while(c >= '0' && c <= '9') c = *fmt++;
        }
        while(c == 'l' || c == 'h' || c == 'z' || c == 'j' || c == 't')
        {
            if(c != 'h') islong = 1;                /* h 是窄化, 变参里本来就是 32 位 */
            c = *fmt++;
        }

        sign = (char)(plus ? '+' : (space ? ' ' : 0));

        switch(c)
        {
            case 'd':
            case 'i':
            {
                long v = islong ? va_arg(ap, long) : (long)va_arg(ap, int);
                if(v < 0) { sign = '-'; v = -v; }
                mini_uint(o, (unsigned long)v, 10ul, 0, width, zero, left, sign);
                break;
            }
            case 'u':
            {
                unsigned long v = islong ? va_arg(ap, unsigned long)
                                         : (unsigned long)va_arg(ap, unsigned int);
                mini_uint(o, v, 10ul, 0, width, zero, left, sign);
                break;
            }
            case 'x':
            case 'X':
            {
                unsigned long v = islong ? va_arg(ap, unsigned long)
                                         : (unsigned long)va_arg(ap, unsigned int);
                mini_uint(o, v, 16ul, c == 'X', width, zero, left, sign);
                break;
            }
            case 'c':
                mini_put(o, (char)va_arg(ap, int));
                break;
            case 's':
            {
                const char *s = va_arg(ap, const char *);
                const char *q;
                int len = 0, p;

                if(!s) s = "(null)";
                for(q = s; *q; q++) len++;
                if(!left && width > len) { p = width - len; while(p-- > 0) mini_put(o, ' '); }
                mini_put_s(o, s);
                if(left  && width > len) { p = width - len; while(p-- > 0) mini_put(o, ' '); }
                break;
            }
            case '%':
                mini_put(o, '%');
                break;
            case 0:                                 /* 格式串以 % 结尾: 原样打出去 */
                mini_put(o, '%');
                return;
            default:                                /* 不认识: 原样打出去, 别吞字符 */
                mini_put(o, '%');
                mini_put(o, c);
                break;
        }
    }
}

int mini_vsnprintf(char *buf, unsigned int n, const char *fmt, va_list ap)
{
    MiniOut o;

    o.buf    = buf;
    o.cap    = n;
    o.n      = 0;
    o.stream = 0;
    o.cn     = 0;

    if(buf && n) buf[0] = 0;
    mini_vformat(&o, fmt, ap);
    if(buf && n) buf[(o.n < n) ? o.n : (n - 1u)] = 0;

    return (int)o.n;
}

int mini_snprintf(char *buf, unsigned int n, const char *fmt, ...)
{
    MiniOut o;
    va_list ap;
    int     r;

    o.buf    = buf;
    o.cap    = n;
    o.n      = 0;
    o.stream = 0;
    o.cn     = 0;

    if(buf && n) buf[0] = 0;
    va_start(ap, fmt);
    mini_vformat(&o, fmt, ap);
    va_end(ap);
    if(buf && n) buf[(o.n < n) ? o.n : (n - 1u)] = 0;

    r = (int)o.n;
    return r;
}

int mini_printf(const char *fmt, ...)
{
    MiniOut o;
    va_list ap;
    int     r;

    o.buf    = 0;
    o.cap    = 0;
    o.n      = 0;
    o.stream = 1;
    o.cn     = 0;

    va_start(ap, fmt);
    mini_vformat(&o, fmt, ap);
    va_end(ap);
    mini_flush(&o);

    r = (int)o.n;
    return r;
}
