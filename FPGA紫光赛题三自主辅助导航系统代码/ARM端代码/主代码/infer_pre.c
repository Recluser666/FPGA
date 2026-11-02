#include "infer_pre.h"

#include <math.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>


#define PRE_MAX_THREADS 1
#define PRE_THREAD_MIN_PIXELS (120 * 1000)

int infer_bytes_per_px(infer_pixfmt_t fmt) {
    switch (fmt) {
        case INFER_PIX_GRAY8:  return 1;
        case INFER_PIX_RGB565: return 2;
        case INFER_PIX_BGR565: return 2;
        case INFER_PIX_RGB888: return 3;
        case INFER_PIX_BGR888: return 3;
        default:               return 0;
    }
}

void infer_pixel_to_rgb(const uint8_t *p, infer_pixfmt_t fmt,
                        uint8_t *r, uint8_t *g, uint8_t *b) {
    switch (fmt) {
        case INFER_PIX_GRAY8:
            *r = *g = *b = p[0];
            break;

        case INFER_PIX_RGB565: {
            uint16_t v = (uint16_t)((p[0] << 8) | p[1]);
            uint8_t r5 = (uint8_t)((v >> 11) & 0x1F);
            uint8_t g6 = (uint8_t)((v >> 5)  & 0x3F);
            uint8_t b5 = (uint8_t)(v & 0x1F);
            *r = (uint8_t)((r5 << 3) | (r5 >> 2));
            *g = (uint8_t)((g6 << 2) | (g6 >> 4));
            *b = (uint8_t)((b5 << 3) | (b5 >> 2));
            break;
        }

        case INFER_PIX_BGR565: {
            uint16_t v = (uint16_t)((p[0] << 8) | p[1]);
            uint8_t b5 = (uint8_t)((v >> 11) & 0x1F);
            uint8_t g6 = (uint8_t)((v >> 5)  & 0x3F);
            uint8_t r5 = (uint8_t)(v & 0x1F);
            *r = (uint8_t)((r5 << 3) | (r5 >> 2));
            *g = (uint8_t)((g6 << 2) | (g6 >> 4));
            *b = (uint8_t)((b5 << 3) | (b5 >> 2));
            break;
        }

        case INFER_PIX_RGB888:
            *r = p[0]; *g = p[1]; *b = p[2];
            break;

        case INFER_PIX_BGR888:
            *b = p[0]; *g = p[1]; *r = p[2];
            break;

        default:
            *r = *g = *b = 0;
            break;
    }
}

/* letterbox：按长边等比缩放再居中补边，保证检测框能原样反算回原图；
 * 缩放比与补边偏移必须与后处理共用同一组参数，否则框会整体偏移。 */
infer_lb_t infer_letterbox_param(int src_w, int src_h, int dst_w, int dst_h) {
    infer_lb_t lb;
    lb.src_w = src_w; lb.src_h = src_h;
    lb.dst_w = dst_w; lb.dst_h = dst_h;

    float r  = (float)dst_w / (float)src_w;
    float r2 = (float)dst_h / (float)src_h;
    if (r2 < r) r = r2;

    lb.ratio = r;
    int nw = (int)lroundf((float)src_w * r);
    int nh = (int)lroundf((float)src_h * r);
    lb.pad_x = (dst_w - nw) / 2;
    lb.pad_y = (dst_h - nh) / 2;
    return lb;
}

typedef struct {
    const uint8_t *src;
    uint8_t       *dst;
    int   src_w, src_h, bpp;
    int   nw, nh;
    int   dst_w, dst_h, pad_x, pad_y;
    float sx_scale, sy_scale;
    infer_pixfmt_t fmt;
    const int32_t *cx0, *cx1, *cwx;
} pre_job_t;

#define PRE_WS   2048
#define PRE_SH   11
#define PRE_HALF (1 << (2 * PRE_SH - 1))

static inline void px_rgb565(const uint8_t *p, int *r, int *g, int *b) {
    uint16_t v = (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
    int r5 = (v >> 11) & 0x1F, g6 = (v >> 5) & 0x3F, b5 = v & 0x1F;
    *r = (r5 << 3) | (r5 >> 2);
    *g = (g6 << 2) | (g6 >> 4);
    *b = (b5 << 3) | (b5 >> 2);
}
static inline void px_rgb888(const uint8_t *p, int *r, int *g, int *b) {
    *r = p[0]; *g = p[1]; *b = p[2];
}

#define DEFINE_PRE_ROWS_I(NAME, LD)                                         \
static void NAME(const pre_job_t *j, int dy0, int dy1) {                    \
    const uint8_t *src  = j->src;                                          \
    uint8_t       *dst  = j->dst;                                          \
    const int src_w = j->src_w, src_h = j->src_h, bpp = j->bpp;            \
    const int nw = j->nw, dst_w = j->dst_w, dst_h = j->dst_h;              \
    const int pad_x = j->pad_x, pad_y = j->pad_y;                          \
    const float sy_scale = j->sy_scale;                                    \
    const int32_t *cx0 = j->cx0, *cx1 = j->cx1, *cwx = j->cwx;             \
    for (int dy = dy0; dy < dy1; ++dy) {                                   \
        int oy = dy + pad_y;                                               \
        if (oy < 0 || oy >= dst_h) continue;                               \
        float fy = (dy + 0.5f) * sy_scale - 0.5f;                           \
        if (fy < 0.0f) fy = 0.0f;                                          \
        if (fy > (float)(src_h - 1)) fy = (float)(src_h - 1);              \
        int y0 = (int)floorf(fy);                                          \
        int y1 = (y0 + 1 < src_h) ? y0 + 1 : src_h - 1;                    \
        int iwy = (int)((fy - (float)y0) * (float)PRE_WS + 0.5f);          \
        int jwy = PRE_WS - iwy;                                            \
        const uint8_t *s0 = src + (size_t)y0 * src_w * bpp;                \
        const uint8_t *s1 = src + (size_t)y1 * src_w * bpp;                \
        uint8_t *d = dst + ((size_t)oy * dst_w + pad_x) * 3;               \
        for (int dx = 0; dx < nw; ++dx) {                                  \
            int x0 = cx0[dx], x1 = cx1[dx];                                \
            int iwx = cwx[dx], jwx = PRE_WS - iwx;                         \
            int r00,g00,b00,r01,g01,b01,r10,g10,b10,r11,g11,b11;           \
            LD(s0 + (size_t)x0 * bpp, &r00,&g00,&b00);                        \
            LD(s0 + (size_t)x1 * bpp, &r01,&g01,&b01);                        \
            LD(s1 + (size_t)x0 * bpp, &r10,&g10,&b10);                        \
            LD(s1 + (size_t)x1 * bpp, &r11,&g11,&b11);                        \
            int a00 = jwx*jwy, a01 = iwx*jwy;                              \
            int a10 = jwx*iwy, a11 = iwx*iwy;                              \
            int rr = a00*r00 + a01*r01 + a10*r10 + a11*r11;                \
            int gg = a00*g00 + a01*g01 + a10*g10 + a11*g11;                \
            int bb = a00*b00 + a01*b01 + a10*b10 + a11*b11;                \
            d[dx*3+0] = (uint8_t)((rr + PRE_HALF) >> (2*PRE_SH));          \
            d[dx*3+1] = (uint8_t)((gg + PRE_HALF) >> (2*PRE_SH));          \
            d[dx*3+2] = (uint8_t)((bb + PRE_HALF) >> (2*PRE_SH));          \
        }                                                                  \
    }                                                                      \
}
DEFINE_PRE_ROWS_I(pre_rows_rgb565, px_rgb565)
DEFINE_PRE_ROWS_I(pre_rows_rgb888, px_rgb888)

typedef struct {
    const pre_job_t *job;
    int dy0, dy1;
} pre_arg_t;

static void pre_rows_generic(const pre_job_t *j, int dy0, int dy1) {
    const uint8_t *src   = j->src;
    uint8_t       *dst   = j->dst;
    const int src_w = j->src_w, src_h = j->src_h, bpp = j->bpp;
    const int nw = j->nw, dst_w = j->dst_w, dst_h = j->dst_h, pad_x = j->pad_x;
    const infer_pixfmt_t fmt = j->fmt;
    const float sx_scale = j->sx_scale, sy_scale = j->sy_scale;

    for (int dy = dy0; dy < dy1; ++dy) {
        int oy = dy + j->pad_y;
        if (oy < 0 || oy >= dst_h) continue;

        float fy = (dy + 0.5f) * sy_scale - 0.5f;
        if (fy < 0.0f) fy = 0.0f;
        if (fy > (float)(src_h - 1)) fy = (float)(src_h - 1);
        int y0 = (int)floorf(fy);
        int y1 = (y0 + 1 < src_h) ? y0 + 1 : src_h - 1;
        float wy = fy - (float)y0;

        const uint8_t *srow0 = src + (size_t)y0 * src_w * bpp;
        const uint8_t *srow1 = src + (size_t)y1 * src_w * bpp;
        uint8_t *drow = dst + ((size_t)oy * dst_w + pad_x) * 3;

        for (int dx = 0; dx < nw; ++dx) {
            int ox = dx + pad_x;
            if (ox < 0 || ox >= dst_w) continue;

            float fx = (dx + 0.5f) * sx_scale - 0.5f;
            if (fx < 0.0f) fx = 0.0f;
            if (fx > (float)(src_w - 1)) fx = (float)(src_w - 1);
            int x0 = (int)floorf(fx);
            int x1 = (x0 + 1 < src_w) ? x0 + 1 : src_w - 1;
            float wx = fx - (float)x0;

            uint8_t r00, g00, b00, r01, g01, b01, r10, g10, b10, r11, g11, b11;
            infer_pixel_to_rgb(srow0 + (size_t)x0 * bpp, fmt, &r00, &g00, &b00);
            infer_pixel_to_rgb(srow0 + (size_t)x1 * bpp, fmt, &r01, &g01, &b01);
            infer_pixel_to_rgb(srow1 + (size_t)x0 * bpp, fmt, &r10, &g10, &b10);
            infer_pixel_to_rgb(srow1 + (size_t)x1 * bpp, fmt, &r11, &g11, &b11);

            float w00 = (1.0f - wx) * (1.0f - wy);
            float w01 = wx * (1.0f - wy);
            float w10 = (1.0f - wx) * wy;
            float w11 = wx * wy;

            drow[ox * 3 + 0] = (uint8_t)lroundf(r00 * w00 + r01 * w01 + r10 * w10 + r11 * w11);
            drow[ox * 3 + 1] = (uint8_t)lroundf(g00 * w00 + g01 * w01 + g10 * w10 + g11 * w11);
            drow[ox * 3 + 2] = (uint8_t)lroundf(b00 * w00 + b01 * w01 + b10 * w10 + b11 * w11);
        }
    }
}

static void pre_rows_dispatch(const pre_job_t *j, int dy0, int dy1) {
    switch (j->fmt) {
        case INFER_PIX_RGB565: pre_rows_rgb565(j, dy0, dy1); break;
        case INFER_PIX_RGB888: pre_rows_rgb888(j, dy0, dy1); break;
        default:               pre_rows_generic(j, dy0, dy1); break;
    }
}

static void *pre_thread(void *p) {
    pre_arg_t *a = (pre_arg_t *)p;
    pre_rows_dispatch(a->job, a->dy0, a->dy1);
    return NULL;
}

int infer_preprocess(const uint8_t *src, int src_w, int src_h, infer_pixfmt_t fmt,
                     uint8_t *dst, int img_size, infer_lb_t *lb_out) {
    return infer_preprocess_ex(src, src_w, src_h, fmt, dst, img_size, img_size, lb_out);
}

int infer_preprocess_ex(const uint8_t *src, int src_w, int src_h, infer_pixfmt_t fmt,
                        uint8_t *dst, int dst_w, int dst_h, infer_lb_t *lb_out) {
    const int bpp = infer_bytes_per_px(fmt);
    if (!src || !dst || bpp <= 0 || src_w <= 0 || src_h <= 0 ||
        dst_w <= 0 || dst_h <= 0)
        return -1;

    infer_lb_t lb = infer_letterbox_param(src_w, src_h, dst_w, dst_h);
    if (lb_out) *lb_out = lb;

    const size_t dst_px = (size_t)dst_w * (size_t)dst_h;

    memset(dst, INFER_PAD_VALUE, dst_px * 3);

    const int nw = (int)lroundf((float)src_w * lb.ratio);
    const int nh = (int)lroundf((float)src_h * lb.ratio);
    if (nw <= 0 || nh <= 0) return -1;

    pre_job_t job;
    job.src = src;      job.dst = dst;
    job.src_w = src_w;  job.src_h = src_h;  job.bpp = bpp;
    job.nw = nw;        job.nh = nh;
    job.dst_w = dst_w;  job.dst_h = dst_h;
    job.pad_x = lb.pad_x; job.pad_y = lb.pad_y;
    job.fmt = fmt;
    job.sx_scale = (float)src_w / (float)nw;
    job.sy_scale = (float)src_h / (float)nh;

    int32_t *lut = (int32_t *)malloc((size_t)nw * 3 * sizeof(int32_t));
    if (!lut) return -1;
    int32_t *cx0 = lut, *cx1 = lut + nw, *cwx = lut + 2 * nw;
    for (int dx = 0; dx < nw; ++dx) {
        float fx = (dx + 0.5f) * job.sx_scale - 0.5f;
        if (fx < 0.0f) fx = 0.0f;
        if (fx > (float)(src_w - 1)) fx = (float)(src_w - 1);
        int ax = (int)floorf(fx);
        cx0[dx] = ax;
        cx1[dx] = (ax + 1 < src_w) ? ax + 1 : src_w - 1;
        cwx[dx] = (int32_t)((fx - (float)ax) * (float)PRE_WS + 0.5f);
    }
    job.cx0 = cx0; job.cx1 = cx1; job.cwx = cwx;

    int nthr = 1;
    const long npix = (long)nw * (long)nh;
    if (npix >= PRE_THREAD_MIN_PIXELS) {
        long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
        if (ncpu > PRE_MAX_THREADS) ncpu = PRE_MAX_THREADS;
        if (ncpu > 1) nthr = (int)ncpu;
    }
    {
        static int env_nthr = -2;
        if (env_nthr == -2) {
            const char *e = getenv("INFER_PRE_THREADS");
            env_nthr = (e && *e) ? atoi(e) : -1;
            if (env_nthr == 0) env_nthr = -1;
        }
        if (env_nthr > 0) nthr = env_nthr;
        if (nthr > PRE_MAX_THREADS) nthr = PRE_MAX_THREADS;
        if (nthr > nh) nthr = nh > 0 ? nh : 1;
    }

    if (nthr == 1) {
        pre_rows_dispatch(&job, 0, nh);
        free(lut);
        return 0;
    }

    pthread_t th[PRE_MAX_THREADS];
    pre_arg_t args[PRE_MAX_THREADS];
    int live[PRE_MAX_THREADS] = {0};
    const int band = nh / nthr;

    for (int i = 0; i < nthr; ++i) {
        args[i].job = &job;
        args[i].dy0 = i * band;
        args[i].dy1 = (i == nthr - 1) ? nh : (i + 1) * band;
        if (pthread_create(&th[i], NULL, pre_thread, &args[i]) == 0) {
            live[i] = 1;
        } else {
            pre_rows_dispatch(&job, args[i].dy0, args[i].dy1);
        }
    }
    for (int i = 0; i < nthr; ++i)
        if (live[i]) pthread_join(th[i], NULL);

    free(lut);
    return 0;
}
