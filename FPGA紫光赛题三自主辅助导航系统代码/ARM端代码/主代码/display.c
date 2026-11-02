#include "display.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <time.h>

#define FB_MAX_THREADS 4
#define FB_DEF_THREADS 2

struct display {
    int      fd;
    uint8_t *mem;
    size_t   mem_len;
    int      fb_w, fb_h, fb_bpp, fb_stride;
    int      src_w, src_h;
    int      order_bgr;
    int      is_real_fb;
    int      usec;
    int      dbg;

    pthread_mutex_t mtx;
};

static void flush_file(display_t *d) {
    if (!d || !d->usec || !d->mem) return;
    if (lseek(d->fd, 0, SEEK_SET) < 0) return;
    size_t off = 0;
    while (off < d->mem_len) {
        ssize_t w = write(d->fd, d->mem + off, d->mem_len - off);
        if (w <= 0) break;
        off += (size_t)w;
    }
}

static double now_ms_d(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static char g_err[256];
const char *display_last_error(void) { return g_err; }

static const uint16_t k_cls565[8] = {
    0x07FF,
    0xFFE0,
    0xF800,
    0x07E0,
    0xFD20,
    0xFFFF, 0xF81F, 0x001F,
};

static inline void put_px(uint8_t *row, int x, int order_bgr,
                          int r, int g, int b) {
    uint8_t *p = row + (size_t)x * 4;
    if (order_bgr) { p[0] = (uint8_t)b; p[1] = (uint8_t)g; p[2] = (uint8_t)r; }
    else           { p[0] = (uint8_t)r; p[1] = (uint8_t)g; p[2] = (uint8_t)b; }
    p[3] = 0xFF;
}

static inline void exp565(uint16_t v, int *r, int *g, int *b) {
    int r5 = (v >> 11) & 0x1F, g6 = (v >> 5) & 0x3F, b5 = v & 0x1F;
    *r = (r5 << 3) | (r5 >> 2);
    *g = (g6 << 2) | (g6 >> 4);
    *b = (b5 << 3) | (b5 >> 2);
}

typedef struct {
    display_t      *d;
    const uint8_t  *src;
    int             y0, y1;
    uint32_t       *linebuf;
    const pp_box_t *boxes;
    int             nbox;
    int             t;
} conv_arg_t;

static void paint_boxes_row(display_t *d, uint32_t *line, int fy,
                            const pp_box_t *boxes, int n, int t, double kx, double ky) {
    (void)d;
    for (int i = 0; i < n; ++i) {
        int cl = boxes[i].cls;
        uint16_t c565 = (cl >= 0 && cl < 8) ? k_cls565[cl] : 0xFFFF;
        int r, g, b; exp565(c565, &r, &g, &b);
        uint32_t px = d->order_bgr ? (0xFF000000u | ((uint32_t)r << 16) |
                                      ((uint32_t)g << 8) | (uint32_t)b)
                                   : (0xFF000000u | ((uint32_t)b << 16) |
                                      ((uint32_t)g << 8) | (uint32_t)r);
        int x1 = (int)(boxes[i].x1 * kx), x2 = (int)(boxes[i].x2 * kx);
        int y1 = (int)(boxes[i].y1 * ky), y2 = (int)(boxes[i].y2 * ky);
        if (x1 > x2) { int tmp = x1; x1 = x2; x2 = tmp; }
        if (y1 > y2) { int tmp = y1; y1 = y2; y2 = tmp; }
        if (fy < y1 || fy > y2) continue;
        const int top = (fy < y1 + t), bot = (fy > y2 - t);
        const int W = d->fb_w;
        if (x1 < 0) x1 = 0;
        if (x2 >= W) x2 = W - 1;
        if (top || bot) {
            for (int x = x1; x <= x2; ++x) line[x] = px;
        } else {
            for (int k = 0; k < t; ++k) {
                if (x1 + k <= x2) line[x1 + k] = px;
                if (x2 - k >= x1) line[x2 - k] = px;
            }
        }
    }
}

static uint32_t g_r5[32], g_g6[64], g_b5[32];
static void init_tabs(void) {
    static int done = 0;
    if (done) return;
    for (int i = 0; i < 32; ++i) g_r5[i] = (uint32_t)((i << 3) | (i >> 2));
    for (int i = 0; i < 64; ++i) g_g6[i] = (uint32_t)((i << 2) | (i >> 4));
    for (int i = 0; i < 32; ++i) g_b5[i] = (uint32_t)((i << 3) | (i >> 2));
    done = 1;
}

/* RGB565 展开成 32bpp：r5/g6/b5 各自查表放大到 8 位，避免逐像素移位，
 * 展开结果直接按帧缓冲的行宽写出。 */
static void conv_row(const uint8_t *restrict s, uint32_t *restrict d, int n, int bgr) {
    if (bgr) {
        for (int x = 0; x < n; ++x) {
            uint16_t v = (uint16_t)(((uint16_t)s[2 * x + 1] << 8) | s[2 * x]);
            d[x] = 0xFF000000u | (g_r5[v >> 11] << 16)
                               | (g_g6[(v >> 5) & 0x3F] << 8)
                               |  g_b5[v & 0x1F];
        }
    } else {
        for (int x = 0; x < n; ++x) {
            uint16_t v = (uint16_t)(((uint16_t)s[2 * x + 1] << 8) | s[2 * x]);
            d[x] = 0xFF000000u | (g_b5[v & 0x1F] << 16)
                               | (g_g6[(v >> 5) & 0x3F] << 8)
                               |  g_r5[v >> 11];
        }
    }
}

static void conv_rows(const conv_arg_t *a) {
    display_t *d = a->d;
    const int sw = d->src_w, sh = d->src_h;
    const int fw = d->fb_w, fh = d->fb_h;
    const int bgr = d->order_bgr;

    const double kx = (double)fw / (double)sw;
    const double ky = (double)fh / (double)sh;

    for (int fy = a->y0; fy < a->y1; ++fy) {
        int sy = (int)((int64_t)fy * sh / fh);
        if (sy >= sh) sy = sh - 1;
        const uint8_t *srow = a->src + (size_t)sy * sw * 2;
        uint32_t *line = a->linebuf;

        if (fw == sw) {
            conv_row(srow, line, fw, bgr);
        } else {
            for (int x = 0; x < fw; ++x) {
                int sx = (int)((int64_t)x * sw / fw);
                if (sx >= sw) sx = sw - 1;
                uint16_t v = (uint16_t)(((uint16_t)srow[2 * sx + 1] << 8) | srow[2 * sx]);
                uint32_t r = g_r5[v >> 11], g = g_g6[(v >> 5) & 0x3F], b = g_b5[v & 0x1F];
                line[x] = bgr ? (0xFF000000u | (r << 16) | (g << 8) | b)
                              : (0xFF000000u | (b << 16) | (g << 8) | r);
            }
        }
        if (a->boxes && a->nbox > 0)
            paint_boxes_row(d, line, fy, a->boxes, a->nbox, a->t, kx, ky);

        memcpy(d->mem + (size_t)fy * d->fb_stride, line, (size_t)fw * 4);
    }
}

static void *conv_thread(void *p) { conv_rows((const conv_arg_t *)p); return NULL; }


typedef struct { char ch; const char *row[7]; } glyph_t;

static const glyph_t k_font[] = {
 {' ', {".....",".....",".....",".....",".....",".....","....."}},
 {'0', {".###.","#...#","#..##","#.#.#","##..#","#...#",".###."}},
 {'1', {"..#..",".##..","..#..","..#..","..#..","..#..",".###."}},
 {'2', {".###.","#...#","....#","...#.","..#..",".#...","#####"}},
 {'3', {"#####","...#.","..#..","...#.","....#","#...#",".###."}},
 {'4', {"...#.","..##.",".#.#.","#..#.","#####","...#.","...#."}},
 {'5', {"#####","#....","####.","....#","....#","#...#",".###."}},
 {'6', {"..##.",".#...","#....","####.","#...#","#...#",".###."}},
 {'7', {"#####","....#","...#.","..#..",".#...",".#...",".#..."}},
 {'8', {".###.","#...#","#...#",".###.","#...#","#...#",".###."}},
 {'9', {".###.","#...#","#...#",".####","....#","...#.",".##.."}},
 {'A', {".###.","#...#","#...#","#####","#...#","#...#","#...#"}},
 {'B', {"####.","#...#","#...#","####.","#...#","#...#","####."}},
 {'C', {".###.","#...#","#....","#....","#....","#...#",".###."}},
 {'F', {"#####","#....","#....","####.","#....","#....","#...."}},
 {'G', {".###.","#...#","#....","#.###","#...#","#...#",".###."}},
 {'M', {"#...#","##.##","#.#.#","#...#","#...#","#...#","#...#"}},
 {'O', {".###.","#...#","#...#","#...#","#...#","#...#",".###."}},
 {'P', {"####.","#...#","#...#","####.","#....","#....","#...."}},
 {'R', {"####.","#...#","#...#","####.","#.#..","#..#.","#...#"}},
 {'S', {".####","#....","#....",".###.","....#","....#","####."}},
 {'X', {"#...#","#...#",".#.#.","..#..",".#.#.","#...#","#...#"}},
 {'Y', {"#...#","#...#",".#.#.","..#..","..#..","..#..","..#.."}},
 {'.', {".....",".....",".....",".....",".....","..##.","..##."}},
 {':', {".....","..##.","..##.",".....","..##.","..##.","....."}},
 {'-', {".....",".....",".....",".###.",".....",".....","....."}},
 {'/', {"....#","....#","...#.","..#..",".#...","#....","#...."}},
 {0, {0}}
};

static const glyph_t *find_glyph(char c) {
    for (const glyph_t *g = k_font; g->ch; ++g)
        if (g->ch == c) return g;
    return NULL;
}

typedef struct { uint32_t *buf; int w, h; } canvas_t;

static inline void cv_put(canvas_t *cv, int x, int y, uint32_t px) {
    if (x < 0 || x >= cv->w || y < 0 || y >= cv->h) return;
    cv->buf[(size_t)y * cv->w + x] = px;
}

static void cv_char(canvas_t *cv, int x, int y, char c, int scale, uint32_t fg, uint32_t bg) {
    const glyph_t *g = find_glyph(c);
    if (!g) g = find_glyph(' ');
    const int W = 5 * scale, H = 7 * scale;
    for (int yy = -1; yy <= H; ++yy)
        for (int xx = -1; xx <= W; ++xx) cv_put(cv, x + xx, y + yy, bg);
    for (int ry = 0; ry < 7; ++ry)
        for (int rx = 0; rx < 5; ++rx) {
            if (g->row[ry][rx] != '#') continue;
            for (int sy = 0; sy < scale; ++sy)
                for (int sx = 0; sx < scale; ++sx)
                    cv_put(cv, x + rx * scale + sx, y + ry * scale + sy, fg);
        }
}

static void cv_text(canvas_t *cv, int x, int y, const char *s, int scale,
                    uint32_t fg, uint32_t bg) {
    for (; *s; ++s, x += 6 * scale) cv_char(cv, x, y, *s, scale, fg, bg);
}

display_t *display_open(const char *path, int src_w, int src_h) {
    if (!path) path = "/dev/fb0";
    display_t *d = (display_t *)calloc(1, sizeof(*d));
    if (!d) return NULL;
    d->fd = -1;
    d->order_bgr = 1;
    d->dbg = getenv("ADAS_DISP_DEBUG") && *getenv("ADAS_DISP_DEBUG") == '1';
    const char *o = getenv("ADAS_FB_ORDER");
    if (o && (!strcmp(o, "rgbx") || !strcmp(o, "rgb"))) d->order_bgr = 0;
    pthread_mutex_init(&d->mtx, NULL);

    d->fd = open(path, O_RDWR);
    if (d->fd < 0) {
        snprintf(g_err, sizeof(g_err), "打不开 %s: %s", path, strerror(errno));
        free(d); return NULL;
    }

    struct fb_var_screeninfo vi;
    struct fb_fix_screeninfo fi;
    /* 行跨距取 fb 自己报的 line_length，而不是 width*4：
     * 有些驱动会做行对齐，用 width*4 画出来的图会斜。 */
    if (ioctl(d->fd, FBIOGET_VSCREENINFO, &vi) == 0 &&
        ioctl(d->fd, FBIOGET_FSCREENINFO, &fi) == 0) {
        d->is_real_fb = 1;
        d->fb_w    = (int)vi.xres;
        d->fb_h    = (int)vi.yres;
        d->fb_bpp  = (int)vi.bits_per_pixel;
        d->fb_stride = (int)fi.line_length;
        d->mem_len = fi.smem_len;
    } else {
        d->is_real_fb = 0;
        d->fb_w = getenv("ADAS_FB_W") ? atoi(getenv("ADAS_FB_W")) : 1920;
        d->fb_h = getenv("ADAS_FB_H") ? atoi(getenv("ADAS_FB_H")) : 1080;
        d->fb_bpp = 32;
        d->fb_stride = d->fb_w * 4;
        d->mem_len = (size_t)d->fb_stride * d->fb_h;
    }
    if (d->fb_bpp != 32) {
        snprintf(g_err, sizeof(g_err),
                 "fb 是 %d bpp，本模块只支持 32bpp（ARGB8888）", d->fb_bpp);
        close(d->fd); free(d); return NULL;
    }

    d->mem = (uint8_t *)mmap(NULL, d->mem_len, PROT_READ | PROT_WRITE,
                             MAP_SHARED, d->fd, 0);
    if (d->mem == MAP_FAILED) {
        d->mem = (uint8_t *)calloc(1, d->mem_len);
        d->usec = 1;
        if (!d->mem) {
            snprintf(g_err, sizeof(g_err), "mmap 失败且 malloc 也失败");
            close(d->fd); free(d); return NULL;
        }
    }

    d->src_w = (src_w > 0) ? src_w : d->fb_w;
    d->src_h = (src_h > 0) ? src_h : d->fb_h;

    init_tabs();
    if (display_x_running())
        fprintf(stderr,
            "[display] ⚠️  检测到 lightdm/X 在运行，它会不停重绘覆盖画面！\n"
            "[display]    先执行：sudo systemctl stop lightdm\n"
            "[display]    退出回桌面：sudo systemctl start lightdm\n");

    fprintf(stderr, "[display] %s %dx%d %dbpp stride=%d  (%s)\n",
            path, d->fb_w, d->fb_h, d->fb_bpp, d->fb_stride,
            d->is_real_fb ? "真 framebuffer" : "裸文件模式");
    return d;
}

void display_close(display_t *d) {
    if (!d) return;
    if (d->mem) {
        if (d->usec) { flush_file(d); free(d->mem); }
        else munmap(d->mem, d->mem_len);
    }
    if (d->fd >= 0) close(d->fd);
    pthread_mutex_destroy(&d->mtx);
    free(d);
}

int display_width(const display_t *d)  { return d ? d->fb_w : 0; }
int display_height(const display_t *d) { return d ? d->fb_h : 0; }

int display_x_running(void) {
    return system("pgrep -x lightdm >/dev/null 2>&1 || pgrep -x Xorg >/dev/null 2>&1") == 0;
}

int display_show(display_t *d, const uint8_t *rgb565, const pp_box_t *boxes, int n,
                 const char *osd) {
    if (!d || !rgb565) return -1;
    pthread_mutex_lock(&d->mtx);

    int nthr = FB_DEF_THREADS;
    { const char *e = getenv("ADAS_DISP_THREADS"); if (e && *e) nthr = atoi(e); }
    if (nthr > FB_MAX_THREADS) nthr = FB_MAX_THREADS;
    if (nthr < 1) nthr = 1;
    if (nthr > d->fb_h) nthr = d->fb_h;
    const double t_conv0 = d->dbg ? now_ms_d() : 0.0;
    const int tpx = (d->fb_w >= 1200) ? 4 : 2;
    if (nthr < 1) nthr = 1;
    pthread_t th[FB_MAX_THREADS];
    conv_arg_t args[FB_MAX_THREADS];
    int live[FB_MAX_THREADS] = {0};
    const int band = d->fb_h / nthr;

    for (int i = 0; i < nthr; ++i) {
        args[i].d = d; args[i].src = rgb565;
        args[i].y0 = i * band;
        args[i].y1 = (i == nthr - 1) ? d->fb_h : (i + 1) * band;
        args[i].boxes = boxes; args[i].nbox = n; args[i].t = tpx;
        args[i].linebuf = (uint32_t *)malloc((size_t)d->fb_w * 4);
        if (args[i].linebuf && pthread_create(&th[i], NULL, conv_thread, &args[i]) == 0) {
            live[i] = 1;
        } else {
            conv_rows(&args[i]);
        }
    }
    for (int i = 0; i < nthr; ++i) if (live[i]) pthread_join(th[i], NULL);
    for (int i = 0; i < nthr; ++i) free(args[i].linebuf);

    const double t_conv1 = d->dbg ? now_ms_d() : 0.0;

    if (osd && *osd) {
        const int sc  = (d->fb_w >= 1200) ? 4 : 2;
        const int y0  = 8;
        const int hgt = 7 * sc + 16;
        int tw = (int)strlen(osd) * 6 * sc + 16;
        if (tw > d->fb_w) tw = d->fb_w;
        const int by = (y0 - 4 < 0) ? 0 : y0 - 4;
        int bh = hgt;
        if (by + bh > d->fb_h) bh = d->fb_h - by;
        if (bh > 0 && tw > 0) {
            uint32_t *band = (uint32_t *)malloc((size_t)tw * (size_t)bh * 4);
            if (band) {
                for (int y = 0; y < bh; ++y)
                    memcpy(band + (size_t)y * tw,
                           d->mem + (size_t)(by + y) * d->fb_stride,
                           (size_t)tw * 4);
                for (int k = 0; k < tw * bh; ++k) {
                    uint32_t v = band[k];
                    uint32_t b = (v & 0xFF) / 3u, g = ((v >> 8) & 0xFF) / 3u,
                             r = ((v >> 16) & 0xFF) / 3u;
                    band[k] = 0xFF000000u | (r << 16) | (g << 8) | b;
                }
                canvas_t cv = { band, tw, bh };
                cv_text(&cv, 12, y0 - by, osd, sc, 0xFFFFFFFFu, 0xFF000000u);
                for (int y = 0; y < bh; ++y)
                    memcpy(d->mem + (size_t)(by + y) * d->fb_stride,
                           band + (size_t)y * tw, (size_t)tw * 4);
                free(band);
            }
        }
    }

    if (d->usec) flush_file(d);
    if (d->dbg) {
        double t1 = now_ms_d();
        fprintf(stderr, "[disp] 转换 %.2fms  叠框+OSD %.2fms  合计 %.2fms (%d 线程)\n",
                t_conv1 - t_conv0, t1 - t_conv1, t1 - t_conv0, nthr);
    }
    pthread_mutex_unlock(&d->mtx);
    return 0;
}
