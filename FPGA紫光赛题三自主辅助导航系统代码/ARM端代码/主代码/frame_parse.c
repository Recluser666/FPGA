#define _GNU_SOURCE
#include "frame_parse.h"
#include "adas_dma_uapi.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

#include <linux/spi/spidev.h>

typedef struct {
    const char *dev;
    uint32_t speed_hz;
    int fd;
} spi_priv_t;

static int spi_open(arm_transport_t *self) {
    spi_priv_t *p = (spi_priv_t *)self->priv;
    p->fd = open(p->dev, O_RDWR);
    if (p->fd < 0) { perror("open spidev"); return -1; }

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    if (ioctl(p->fd, SPI_IOC_WR_MODE, &mode) < 0) { perror("SPI_IOC_WR_MODE"); return -2; }
    if (ioctl(p->fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) { perror("SPI_IOC_WR_BITS"); return -3; }
    if (ioctl(p->fd, SPI_IOC_WR_MAX_SPEED_HZ, &p->speed_hz) < 0) { perror("SPI_IOC_WR_SPEED"); return -4; }
    return 0;
}

static int spi_read(arm_transport_t *self, uint8_t *buf, size_t len) {
    spi_priv_t *p = (spi_priv_t *)self->priv;
    if (p->fd < 0) return -1;
    const size_t CHUNK = 4096;
    size_t done = 0;
    while (done < len) {
        size_t n = (len - done > CHUNK) ? CHUNK : (len - done);
        struct spi_ioc_transfer tr;
        memset(&tr, 0, sizeof(tr));
        tr.tx_buf = 0;
        tr.rx_buf = (unsigned long)(buf + done);
        tr.len = (uint32_t)n;
        tr.speed_hz = p->speed_hz;
        tr.delay_usecs = 0;
        tr.bits_per_word = 8;
        int r = ioctl(p->fd, SPI_IOC_MESSAGE(1), &tr);
        if (r < 1) { perror("SPI_IOC_MESSAGE"); return -1; }
        done += n;
    }
    return (int)done;
}

static void spi_close(arm_transport_t *self) {
    spi_priv_t *p = (spi_priv_t *)self->priv;
    if (p->fd >= 0) close(p->fd);
    p->fd = -1;
}

arm_transport_t *arm_transport_spi_new(const char *dev, uint32_t speed_hz) {
    arm_transport_t *t = (arm_transport_t *)calloc(1, sizeof(*t));
    spi_priv_t *p = (spi_priv_t *)calloc(1, sizeof(*p));
    if (!t || !p) { free(t); free(p); return NULL; }
    p->dev = dev; p->speed_hz = speed_hz ? speed_hz : 50000000; p->fd = -1;
    t->name = "spidev";
    t->open = spi_open; t->read = spi_read; t->close = spi_close; t->priv = p;
    return t;
}

typedef struct { const char *path; int fd; uint64_t max_frames; uint64_t served; } file_priv_t;

static int file_open(arm_transport_t *self) {
    file_priv_t *p = (file_priv_t *)self->priv;
    p->fd = open(p->path, O_RDONLY);
    if (p->fd < 0) { perror("open 回环文件"); return -1; }
    return 0;
}

static int file_read(arm_transport_t *self, uint8_t *buf, size_t len) {
    file_priv_t *p = (file_priv_t *)self->priv;
    if (p->max_frames > 0 && p->served >= p->max_frames) {
        usleep(2000);
        return 0;
    }
    size_t done = 0;
    while (done < len) {
        ssize_t r = read(p->fd, buf + done, len - done);
        if (r == 0) {
            if (lseek(p->fd, 0, SEEK_SET) < 0) return -1;
            continue;
        }
        if (r < 0) {
            if (errno == EINTR) continue;
            perror("read 回环文件");
            return -1;
        }
        done += (size_t)r;
    }
    p->served++;
    return (int)done;
}

static void file_close(arm_transport_t *self) {
    file_priv_t *p = (file_priv_t *)self->priv;
    if (p->fd >= 0) close(p->fd);
    p->fd = -1;
}

arm_transport_t *arm_transport_file_new_ex(const char *path, uint64_t max_frames) {
    arm_transport_t *t = (arm_transport_t *)calloc(1, sizeof(*t));
    file_priv_t *p = (file_priv_t *)calloc(1, sizeof(*p));
    if (!t || !p) { free(t); free(p); return NULL; }
    p->path = path; p->fd = -1; p->max_frames = max_frames; p->served = 0;
    t->name = max_frames ? "file-finite" : "file-loopback";
    t->open = file_open; t->read = file_read; t->close = file_close; t->priv = p;
    return t;
}

arm_transport_t *arm_transport_file_new(const char *path) {
    return arm_transport_file_new_ex(path, 0);
}

typedef struct {
    const char *path;
    int         fd;
    int         w, h, bpp;
    frame_pixfmt_t fmt;
    uint32_t    payload;
    uint16_t    seq;
    int         is_stdin;
    int         loop;
    double      t_next;
    double      period;
} raw_priv_t;

static int raw_open(arm_transport_t *self) {
    raw_priv_t *p = (raw_priv_t *)self->priv;
    if (p->fd >= 0) return 0;
    if (!strcmp(p->path, "-") || !strcmp(p->path, "/dev/stdin")) {
        p->fd = 0; p->is_stdin = 1;
    } else {
        p->fd = open(p->path, O_RDONLY);
        if (p->fd < 0) {
            fprintf(stderr, "[raw] 打不开 %s: %s\n", p->path, strerror(errno));
            return -1;
        }
    }
    p->fd = p->fd;
    fprintf(stderr, "[raw] %s  %dx%d bpp=%d  每帧 %u 字节（裸流，无帧头）"
                    "  节拍 %s\n",
            p->is_stdin ? "stdin" : p->path, p->w, p->h, p->bpp, p->payload,
            p->period > 0 ? "已启用" : "无（要限速设 ADAS_RAW_FPS）");
    return 0;
}

static size_t read_full(int fd, uint8_t *buf, size_t len) {
    size_t off = 0;
    while (off < len) {
        ssize_t n = read(fd, buf + off, len - off);
        if (n > 0) { off += (size_t)n; continue; }
        if (n == 0) break;
        if (errno == EINTR) continue;
        break;
    }
    return off;
}

static int raw_read(arm_transport_t *self, uint8_t *buf, size_t len) {
    raw_priv_t *p = (raw_priv_t *)self->priv;
    const size_t need = (size_t)FRAME_HDR_LEN + p->payload + FRAME_CRC_LEN;
    if (len < need) return -1;
    if (p->fd < 0 && raw_open(self) != 0) return -1;

    uint8_t *pix = buf + FRAME_HDR_LEN;
    size_t got = read_full(p->fd, pix, p->payload);
    if (got == 0) {
        if (p->loop && !p->is_stdin && lseek(p->fd, 0, SEEK_SET) == 0) {
            p->seq = 0;
            got = read_full(p->fd, pix, p->payload);
        }
        if (got == 0) return 0;
    }
    if (got < p->payload) {
        fprintf(stderr, "[raw] 流末尾不足一帧（%zu/%u 字节），停止\n", got, p->payload);
        return 0;
    }

    buf[0] = FRAME_MAGIC_0;
    buf[1] = FRAME_MAGIC_1;
    buf[2] = (uint8_t)(p->seq >> 8);  buf[3] = (uint8_t)(p->seq & 0xFF);
    buf[4] = (uint8_t)(p->w  >> 8);   buf[5] = (uint8_t)(p->w  & 0xFF);
    buf[6] = (uint8_t)(p->h  >> 8);   buf[7] = (uint8_t)(p->h  & 0xFF);
    buf[8]  = (uint8_t)(p->payload >> 24); buf[9]  = (uint8_t)(p->payload >> 16);
    buf[10] = (uint8_t)(p->payload >> 8);  buf[11] = (uint8_t)(p->payload & 0xFF);
    buf[12] = (uint8_t)((int)p->fmt >> 8); buf[13] = (uint8_t)((int)p->fmt & 0xFF);

    buf[FRAME_HDR_LEN + p->payload]     = 0;
    buf[FRAME_HDR_LEN + p->payload + 1] = 0;

    p->seq++;

    if (p->period > 0.0) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double now = (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
        if (p->t_next < now) p->t_next = now;
        p->t_next += p->period;
        double d = p->t_next - now;
        if (d > 0.0) {
            struct timespec sl;
            sl.tv_sec = (time_t)d;
            sl.tv_nsec = (long)((d - (double)sl.tv_sec) * 1e9);
            nanosleep(&sl, NULL);
        }
    }
    return (int)need;
}

static void raw_close(arm_transport_t *self) {
    raw_priv_t *p = (raw_priv_t *)self->priv;
    if (p && p->fd >= 0 && !p->is_stdin) close(p->fd);
    if (p) p->fd = -1;
}

arm_transport_t *arm_transport_raw_new(const char *path, int w, int h, frame_pixfmt_t fmt) {
    if (!path || w <= 0 || h <= 0) return NULL;
    const int bpp = frame_bytes_per_px(fmt);
    if (bpp <= 0) return NULL;
    arm_transport_t *t = (arm_transport_t *)calloc(1, sizeof(*t));
    raw_priv_t *p = (raw_priv_t *)calloc(1, sizeof(*p));
    if (!t || !p) { free(t); free(p); return NULL; }
    p->path = path; p->fd = -1; p->w = w; p->h = h; p->bpp = bpp; p->fmt = fmt;
    p->loop = (strcmp(path, "-") && strcmp(path, "/dev/stdin"));
    {
        const char *e = getenv("ADAS_RAW_FPS");
        double f = (e && *e) ? atof(e) : 0.0;
        p->period = (f > 0.5) ? 1.0 / f : 0.0;
        p->t_next = 0.0;
    }
    p->payload = (uint32_t)((size_t)w * (size_t)h * (size_t)bpp);
    t->name = "raw-stream";
    t->open = raw_open; t->read = raw_read; t->close = raw_close; t->priv = p;
    return t;
}


#define DMA_BAR_CMD_ADDR    0x110u
#define DMA_BAR_CLEAR_ADDR  0x130u
#define DMA_FOOTER_MAGIC    0x53414441u
#define DMA_FOOTER_MAGIC_SWAP 0x41444153u
#define DMA_FOOTER_LEN      64u

static int dma_footer_fetch(const uint8_t *base, uint32_t payload_bytes,
                            uint32_t ft[16])
{
    int k, swapped = 0;

    memcpy(ft, base + payload_bytes, DMA_FOOTER_LEN);
    if (ft[0] == DMA_FOOTER_MAGIC_SWAP) {
        for (k = 0; k < 16; k++) {
            uint32_t v = ft[k];
            ft[k] = ((v & 0x000000FFu) << 24) | ((v & 0x0000FF00u) << 8) |
                    ((v & 0x00FF0000u) >> 8)  | ((v & 0xFF000000u) >> 24);
        }
        swapped = 1;
    }
    return (ft[0] == DMA_FOOTER_MAGIC) ? (swapped ? 2 : 1) : 0;
}

static inline void dma_put_be16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v;
}
static inline void dma_put_be32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);  p[3] = (uint8_t)v;
}

typedef struct {
    const char     *dev_path;
    const char     *bar_path;
    int             dma_fd;
    uint8_t        *bufs;
    uint32_t        nbufs;
    uint32_t        bufsize;
    uint64_t        dma_addr[ADAS_DMA_MAX_BUFS];
    uint32_t        payload_bytes;
    int             frame_w, frame_h;
    frame_pixfmt_t  fmt;
    int             swap_mode;
    int             bar_fd;
    volatile uint8_t *bar;
    size_t          bar_len;
    uint32_t        last_seq[ADAS_DMA_MAX_BUFS];
    uint64_t        frames;
    int             use[ADAS_DMA_MAX_BUFS];
    int             nuse;
    int             warned_swap;
    uint64_t        tears;
    double          geo_stride;
    int             geo_rows, geo_srcw, geo_xoff;
    int             deskew;
    int16_t         deskew_prev[256];
    int             rot;
    int             track;
    int             acc_dx;
    int             have_prof;
    uint32_t       *prof_cur;
    uint32_t       *prof_prv;
} dma_priv_t;

static void dma_pick_buffers(dma_priv_t *p);
static void dma_pci_enable_mem(const char *bar_path);

static inline int luma565_s(uint16_t v)
{
    return ((v >> 11) & 0x1F) * 2 + ((v >> 5) & 0x3F) * 3 + (v & 0x1F) * 2;
}

static void dma_deskew(uint8_t *dst, const uint16_t *src, int W, int H,
                       int nominal, uint32_t bufsize_px, int16_t *prev)
{
    const int NS = 64;
    const int COARSE = 16;
    const int FINE = 15;
    size_t off = 0;
    int y, j, k;

    for (y = 0; y < H; y++) {
        size_t start;
        int    best_d = 0;

        if (y == 0) {
            start = 0;
        } else {
            long best = -1;
            int  d;
            for (d = 0; d < W; d += COARSE) {
                long sad = 0;
                for (j = 0; j < NS; j++) {
                    int x = (int)((long)j * W / NS);
                    int diff = luma565_s(src[off + (size_t)((x + d) % W)]) - prev[j];
                    sad += diff < 0 ? -diff : diff;
                }
                if (best < 0 || sad < best) { best = sad; best_d = d; }
            }
            for (d = best_d - FINE; d <= best_d + FINE; d++) {
                int dd = ((d % W) + W) % W;
                long sad = 0;
                if (dd == best_d) continue;
                for (j = 0; j < NS; j++) {
                    int x = (int)((long)j * W / NS);
                    int diff = luma565_s(src[off + (size_t)((x + dd) % W)]) - prev[j];
                    sad += diff < 0 ? -diff : diff;
                }
                if (sad < best) { best = sad; best_d = dd; }
            }
            start = off;
            best_d = best_d;
        }

        if (start + (size_t)W >= bufsize_px) break;

        {
            uint8_t *drow = dst + (size_t)y * W * 2;
            for (k = 0; k < W; k++) {
                uint16_t v = src[start + (size_t)((k + best_d) % W)];
                drow[k * 2 + 0] = (uint8_t)(v & 0xFF);
                drow[k * 2 + 1] = (uint8_t)(v >> 8);
            }
        }
        for (j = 0; j < NS; j++) {
            int x = (int)((long)j * W / NS);
            prev[j] = (int16_t)luma565_s(src[start + (size_t)((x + best_d) % W)]);
        }
        off = start + (size_t)nominal;
    }
}

static void dma_geo_resample(uint8_t *dst, const uint8_t *base,
                             int W, int H, double stride,
                             int srcrows, int srcw, int xoff, uint32_t bufsize)
{
    int y, x;

    for (y = 0; y < H; y++) {
        long   sy = (long)y * srcrows / H;
        double rowoff = (double)sy * stride + 0.5 + (double)xoff;
        uint8_t *drow = dst + (size_t)y * W * 2;
        const uint8_t *srow;

        if (rowoff < 0 || (size_t)rowoff + (size_t)srcw > bufsize / 2) {
            memset(drow, 0, (size_t)W * 2);
            continue;
        }
        srow = base + (size_t)rowoff * 2;
        for (x = 0; x < W; x++) {
            int sx = (int)((long)x * srcw / W);
            drow[x * 2 + 0] = srow[sx * 2 + 0];
            drow[x * 2 + 1] = srow[sx * 2 + 1];
        }
    }
}

static void dma_rot_rows(uint8_t *img, int W, int H, int rot)
{
    int y, x;
    uint8_t tmp[4096];

    if (rot == 0 || W <= 0 || W > 2048) return;
    rot %= W;
    if (rot < 0) rot += W;
    if (rot == 0) return;

    for (y = 0; y < H; y++) {
        uint8_t *row = img + (size_t)y * W * 2;

        memcpy(tmp, row, (size_t)W * 2);
        for (x = 0; x < W; x++) {
            int sx = x - rot;
            if (sx < 0) sx += W;
            row[x * 2 + 0] = tmp[sx * 2 + 0];
            row[x * 2 + 1] = tmp[sx * 2 + 1];
        }
    }
}


static void dma_prof_cols(const uint8_t *img, int W, int H, uint32_t *prof)
{
    int x, y;
    for (x = 0; x < W; x++) prof[x] = 0;
    for (y = 0; y < H; y += 4) {
        const uint8_t *row = img + (size_t)y * W * 2;
        for (x = 0; x < W; x++) {
            uint16_t v = (uint16_t)(row[x * 2] | (row[x * 2 + 1] << 8));
            prof[x] += (uint32_t)(((v >> 11) & 0x1F) * 2 + ((v >> 5) & 0x3F) * 3 + (v & 0x1F) * 2);
        }
    }
}

static int dma_find_dx(const uint32_t *cur, const uint32_t *prv, int W)
{
    int d, i, best_d = 0;
    unsigned long long best = 0;
    for (d = 0; d < W; d += 4) {
        unsigned long long sc = 0;
        for (i = 0; i < W; i += 8) {
            int j = i - d; if (j < 0) j += W;
            sc += (unsigned long long)cur[i] * prv[j];
        }
        if (sc > best) { best = sc; best_d = d; }
    }
    return (best_d > W / 2) ? best_d - W : best_d;
}

static void dma_copy_pixels(uint8_t *dst, const uint8_t *src, size_t n, int swap_mode)
{
    if (swap_mode == 0) {
        memcpy(dst, src, n);
        return;
    }
    size_t i = 0;

    if (swap_mode == 3) {
        for (; i + 16 <= n; i += 16) {
            const uint8_t *sp = src + i;
            uint8_t       *dp = dst + i;
            int k;
            for (k = 0; k < 8; k++) {
                dp[k * 2 + 0] = sp[(7 - k) * 2 + 0];
                dp[k * 2 + 1] = sp[(7 - k) * 2 + 1];
            }
        }
        if (i < n) memcpy(dst + i, src + i, n - i);
        return;
    }

    if (swap_mode == 4) {
        for (; i + 16 <= n; i += 16) {
            const uint8_t *sp = src + i;
            uint8_t       *dp = dst + i;
            int k;
            for (k = 0; k < 8; k++) {
                int sj = (6 - k) & 7;
                dp[k * 2 + 0] = sp[sj * 2 + 0];
                dp[k * 2 + 1] = sp[sj * 2 + 1];
            }
        }
        if (i < n) memcpy(dst + i, src + i, n - i);
        return;
    }

    if (swap_mode == 2) {
        for (; i + 4 <= n; i += 4) {
            dst[i + 0] = src[i + 3];
            dst[i + 1] = src[i + 2];
            dst[i + 2] = src[i + 1];
            dst[i + 3] = src[i + 0];
        }
    } else {
        for (; i + 4 <= n; i += 4) {
            dst[i + 0] = src[i + 2];
            dst[i + 1] = src[i + 3];
            dst[i + 2] = src[i + 0];
            dst[i + 3] = src[i + 1];
        }
    }
    if (i < n) memcpy(dst + i, src + i, n - i);
}

static int dma_open(arm_transport_t *self)
{
    dma_priv_t *p = (dma_priv_t *)self->priv;
    struct adas_dma_info info;
    size_t total;
    uint32_t i;
    const char *env;

    p->dma_fd = open(p->dev_path, O_RDWR);
    if (p->dma_fd < 0) {
        fprintf(stderr, "[dma] ❌ 打不开 %s（先 insmod adas_dma.ko）: %s\n",
                p->dev_path, strerror(errno));
        return -1;
    }
    memset(&info, 0, sizeof(info));
    if (ioctl(p->dma_fd, ADAS_DMA_GET_INFO, &info) < 0) {
        fprintf(stderr, "[dma] ❌ ADAS_DMA_GET_INFO 失败: %s\n", strerror(errno));
        close(p->dma_fd); p->dma_fd = -1;
        return -2;
    }
    if (info.nbufs < 1 || info.nbufs > ADAS_DMA_MAX_BUFS) {
        fprintf(stderr, "[dma] ❌ 缓冲区数量异常: %u\n", info.nbufs);
        return -3;
    }
    if (info.bufsize < p->payload_bytes + DMA_FOOTER_LEN) {
        fprintf(stderr, "[dma] ❌ 缓冲区不够大: 驱动 %u 字节 < 需要 %u + %u\n",
                info.bufsize, p->payload_bytes, DMA_FOOTER_LEN);
        return -3;
    }

    p->nbufs   = info.nbufs;
    p->bufsize = info.bufsize;
    for (i = 0; i < p->nbufs; i++) {
        p->dma_addr[i] = info.dma_addr[i];
        p->last_seq[i] = 0;
    }

    total = (size_t)p->nbufs * p->bufsize;
    p->bufs = (uint8_t *)mmap(NULL, total, PROT_READ | PROT_WRITE,
                              MAP_SHARED, p->dma_fd, 0);
    if (p->bufs == MAP_FAILED) {
        fprintf(stderr, "[dma] ❌ mmap 缓冲区失败: %s\n", strerror(errno));
        p->bufs = NULL;
        return -4;
    }

    for (i = 0; i < p->nbufs; i++) {
        struct adas_dma_buf_arg a = { .idx = i, .reserved = 0 };
        uint8_t *base = p->bufs + (size_t)i * p->bufsize;
        uint32_t *ft   = (uint32_t *)(void *)(base + p->payload_bytes);

        (void)ioctl(p->dma_fd, ADAS_DMA_SYNC_CPU, &a);
        ft[0] = 0;
        ft[1] = 0;
        (void)ioctl(p->dma_fd, ADAS_DMA_SYNC_DEV, &a);
        p->last_seq[i] = 0;
    }

    {
        const char *e;
        p->geo_stride = 0; p->geo_rows = 0; p->geo_srcw = 0; p->geo_xoff = 0;
        p->rot = 0;
        e = getenv("ADAS_DMA_STRIDE");  if (e && *e) p->geo_stride = atof(e);
        e = getenv("ADAS_DMA_SRCROWS"); if (e && *e) p->geo_rows   = atoi(e);
        e = getenv("ADAS_DMA_SRCW");    if (e && *e) p->geo_srcw   = atoi(e);
        e = getenv("ADAS_DMA_XOFF");    if (e && *e) p->geo_xoff   = atoi(e);
        e = getenv("ADAS_DMA_ROT");     if (e && *e) p->rot        = atoi(e);
        e = getenv("ADAS_DMA_TRACK");   if (e && *e == '1') {
            p->track = 1;
            p->prof_cur = (uint32_t *)calloc((size_t)p->frame_w, sizeof(uint32_t));
            p->prof_prv = (uint32_t *)calloc((size_t)p->frame_w, sizeof(uint32_t));
            if (!p->prof_cur || !p->prof_prv) { p->track = 0; }
            else fprintf(stderr, "[dma] 逐帧横向漂移跟踪已启用（ADAS_DMA_TRACK=1）\n");
        }
        if (p->geo_stride <= 0) p->geo_stride = (double)p->frame_w;
        e = getenv("ADAS_DMA_DESKEW");
        p->deskew = (e && *e == '1');
        if (p->deskew)
            fprintf(stderr, "[dma] 逐行自动对齐（deskew）已启用\n");
        else if (p->geo_rows > 0 && p->geo_srcw > 0)
            fprintf(stderr, "[dma] 几何校正启用：跨距 %.2f 像素，源 %d 行，源宽 %d，行内偏移 %d\n",
                    p->geo_stride, p->geo_rows, p->geo_srcw, p->geo_xoff);
    }

    env = getenv("ADAS_DMA_SWAP");
    p->swap_mode = env ? atoi(env) : 4;

    fprintf(stderr, "[dma] ✅ %u 块缓冲区 x %u 字节，payload=%u，swap_mode=%d\n",
            p->nbufs, p->bufsize, p->payload_bytes, p->swap_mode);
    for (i = 0; i < p->nbufs; i++)
        fprintf(stderr, "[dma]    buf[%u] FPGA 地址 = 0x%08llx\n",
                i, (unsigned long long)p->dma_addr[i]);

    dma_pick_buffers(p);
    if (p->nuse == 0)
        return -1;

    {
        const char *nocmd = getenv("ADAS_DMA_NOCMD");
        if (nocmd && *nocmd == '1') {
            fprintf(stderr, "[dma] ADAS_DMA_NOCMD=1：跳过 BAR 命令下发（地址由 FPGA 侧写死）\n");
            fprintf(stderr, "[dma]    请确认 FPGA 用的就是上面这 4 个地址，直接开始轮询 footer\n");
            return 0;
        }
    }
    if (!p->bar_path) {
        fprintf(stderr, "[dma] ⚠️ 未指定 BAR 路径，跳过地址下发（只能测读取侧）\n");
        return 0;
    }
    dma_pci_enable_mem(p->bar_path);

    p->bar_fd = open(p->bar_path, O_RDWR | O_SYNC);
    if (p->bar_fd < 0) {
        fprintf(stderr, "[dma] ⚠️ 打不开 BAR %s: %s（FPGA 没接？）\n",
                p->bar_path, strerror(errno));
        return 0;
    }
    {
        struct stat st;
        if (fstat(p->bar_fd, &st) != 0 || st.st_size <= (off_t)(DMA_BAR_CLEAR_ADDR + 4)) {
            fprintf(stderr, "[dma] ⚠️ BAR 大小异常: %lld 字节\n", (long long)st.st_size);
        }
        p->bar_len = (size_t)st.st_size;
    }
    p->bar = (volatile uint8_t *)mmap(NULL, p->bar_len, PROT_READ | PROT_WRITE,
                                      MAP_SHARED, p->bar_fd, 0);
    if (p->bar == MAP_FAILED) {
        fprintf(stderr, "[dma] ❌ mmap BAR 失败: %s\n", strerror(errno));
        p->bar = NULL;
        return 0;
    }

    {
        int         gap_ms = 200;
        const char *g      = getenv("ADAS_DMA_GAP_MS");
        useconds_t  gap_us;

        if (g && *g) { int v = atoi(g); if (v > 0) gap_ms = v; }
        gap_us = (useconds_t)gap_ms * 1000u;

        fprintf(stderr, "[dma] 地址下发间隔 %d ms（ADAS_DMA_GAP_MS 可调）\n", gap_ms);

        *(volatile uint32_t *)(p->bar + DMA_BAR_CLEAR_ADDR) = 1u;
        fprintf(stderr, "[dma]    已写 BAR+0x%03X = 1，等 %d ms\n",
                DMA_BAR_CLEAR_ADDR, gap_ms);
        usleep(gap_us);

        for (i = 0; i < 4; i++) {
            int      idx = p->use[i % p->nuse];
            uint32_t a   = (uint32_t)p->dma_addr[idx];
            *(volatile uint32_t *)(p->bar + DMA_BAR_CMD_ADDR) = a;
            fprintf(stderr, "[dma]    -> 槽%u = buf%d = 0x%08x\n", i, idx, a);
            usleep(gap_us);
        }
    }
    fprintf(stderr, "[dma] ✅ 已向 FPGA 下发 4 个缓冲区地址（BAR+0x%03X）\n",
            DMA_BAR_CMD_ADDR);
    return 0;
}

static void dma_pci_enable_mem(const char *bar_path)
{
    char     cfg[512];
    const char *p;
    size_t   n;
    int      fd;
    uint16_t cmd;

    if (!bar_path) return;
    p = strstr(bar_path, "/resource");
    if (!p) return;
    n = (size_t)(p - bar_path);
    if (n + 8 >= sizeof cfg) return;
    memcpy(cfg, bar_path, n);
    memcpy(cfg + n, "/config", 8);

    fd = open(cfg, O_RDWR);
    if (fd < 0) return;
    if (pread(fd, &cmd, 2, 0x04) == 2) {
        if ((cmd & 0x0006) != 0x0006) {
            cmd |= 0x0006;
            if (pwrite(fd, &cmd, 2, 0x04) == 2)
                fprintf(stderr, "[dma] 已使能 COMMAND 的 Mem Space + Bus Master（→ 0x%04X）\n", cmd);
            else
                fprintf(stderr, "[dma] ⚠️ 写 COMMAND 失败: %s\n", strerror(errno));
        } else {
            fprintf(stderr, "[dma] COMMAND 已经是 0x%04X（Mem+Master 已开）\n", cmd);
        }
    }
    close(fd);
}

static int phys_unsafe(uint64_t phys, uint64_t len)
{
    FILE *f = fopen("/proc/iomem", "r");
    char  line[512];
    if (!f) return 0;
    while (fgets(line, sizeof line, f)) {
        unsigned long long a, b;
        char  *colon;
        const char *name;
        if (sscanf(line, " %llx-%llx", &a, &b) != 2) continue;
        colon = strchr(line, ':');
        if (!colon) continue;
        name = colon + 1;
        while (*name == ' ') name++;
        if (!strstr(name, "Kernel") && strncmp(name, "reserved", 8) != 0)
            continue;
        if (phys <= (uint64_t)b && (uint64_t)a < phys + len)
            { fclose(f); return 1; }
    }
    fclose(f);
    return 0;
}

static void dma_pick_buffers(dma_priv_t *p)
{
    const char *use = getenv("ADAS_DMA_USE");
    uint32_t i;
    p->nuse = 0;

    if (use && *use) {
        const char *s = use;
        while (*s && p->nuse < ADAS_DMA_MAX_BUFS) {
            char *end;
            long  v = strtol(s, &end, 0);
            if (end == s) break;
            if (v >= 0 && v < (long)p->nbufs)
                p->use[p->nuse++] = (int)v;
            s = (*end == ',') ? end + 1 : end;
        }
        if (p->nuse > 0) {
            fprintf(stderr, "[dma] ADAS_DMA_USE=%s：只用 buf", use);
            for (i = 0; i < (uint32_t)p->nuse; i++) fprintf(stderr, " %d", p->use[i]);
            fprintf(stderr, "\n");
            return;
        }
        fprintf(stderr, "[dma] ⚠️ ADAS_DMA_USE=%s 解析不出有效下标，改为自动挑选\n", use);
    }

    for (i = 0; i < p->nbufs; i++) {
        if (phys_unsafe(p->dma_addr[i], p->bufsize)) {
            fprintf(stderr,
                    "[dma] ⚠️ buf[%u] 物理地址 0x%08llx 落在内核/保留区，已自动跳过\n"
                    "[dma]    （FPGA 往这里写会直接打崩内核）\n",
                    i, (unsigned long long)p->dma_addr[i]);
            continue;
        }
        p->use[p->nuse++] = (int)i;
    }
    if (p->nuse == 0) {
        fprintf(stderr, "[dma] ❌ 4 块缓冲区全在内核/保留区，拒绝下发地址\n");
        return;
    }
    fprintf(stderr, "[dma] 自动挑选出 %d 块可用缓冲区：", p->nuse);
    for (i = 0; i < (uint32_t)p->nuse; i++) fprintf(stderr, " %d", p->use[i]);
    fprintf(stderr, "\n");
}

static int dma_read(arm_transport_t *self, uint8_t *buf, size_t len)
{
    dma_priv_t *p = (dma_priv_t *)self->priv;
    const size_t need = (size_t)FRAME_HDR_LEN + p->payload_bytes + FRAME_CRC_LEN;
    uint32_t i;

    if (len < need) return -1;

    uint32_t best_i = p->nbufs, best_seq = 0;
    int      best_swap = 0;
    int k;

    for (k = 0; k < p->nuse; k++) {
        struct adas_dma_buf_arg a;
        uint8_t *base;
        uint32_t ft[16];
        int sw;

        i = (uint32_t)p->use[k];
        a.idx = i; a.reserved = 0;
        base = p->bufs + (size_t)i * p->bufsize;
        (void)ioctl(p->dma_fd, ADAS_DMA_SYNC_CPU, &a);
        sw = dma_footer_fetch(base, p->payload_bytes, ft);
        if (!sw || ft[1] == 0)            { (void)ioctl(p->dma_fd, ADAS_DMA_SYNC_DEV, &a); continue; }
        if (ft[1] == p->last_seq[i])      { (void)ioctl(p->dma_fd, ADAS_DMA_SYNC_DEV, &a); continue; }
        if (best_i == p->nbufs || ft[1] > best_seq) {
            best_i = i; best_seq = ft[1]; best_swap = sw;
        }
        (void)ioctl(p->dma_fd, ADAS_DMA_SYNC_DEV, &a);
    }

    if (best_i < p->nbufs) {
        i = best_i;
        struct adas_dma_buf_arg a = { .idx = i, .reserved = 0 };
        uint8_t *base = p->bufs + (size_t)i * p->bufsize;
        uint32_t ft[16];
        uint32_t seq;
        uint16_t w, h, fmt;
        uint16_t crc;

        (void)ioctl(p->dma_fd, ADAS_DMA_SYNC_CPU, &a);
        (void)dma_footer_fetch(base, p->payload_bytes, ft);
        seq = ft[1];

        if (best_swap == 2 && !p->warned_swap) {
            fprintf(stderr, "[dma] ℹ️ footer 字节序是反的（每 DW 4 字节倒序），"
                            "已自动归一化；像素侧请配 ADAS_DMA_SWAP=2\n");
            p->warned_swap = 1;
        }

        w   = (uint16_t)(ft[2] & 0xFFFFu);
        h   = (uint16_t)((ft[2] >> 16) & 0xFFFFu);
        fmt = (uint16_t)(ft[3] & 0xFFFFu);

        if (w != (uint16_t)p->frame_w || h != (uint16_t)p->frame_h ||
            fmt != (uint16_t)p->fmt) {
            fprintf(stderr, "[dma] ⚠️ footer 与配置不一致: %ux%u fmt=%u（期望 %dx%d fmt=%d），丢弃\n",
                    w, h, fmt, p->frame_w, p->frame_h, (int)p->fmt);
            p->last_seq[i] = seq;
            (void)ioctl(p->dma_fd, ADAS_DMA_SYNC_DEV, &a);
            usleep(500);
            return 0;
        }

        buf[0] = FRAME_MAGIC_0;
        buf[1] = FRAME_MAGIC_1;
        dma_put_be16(buf + 2, (uint16_t)seq);
        dma_put_be16(buf + 4, w);
        dma_put_be16(buf + 6, h);
        dma_put_be32(buf + 8, p->payload_bytes);
        {
            uint16_t fflags = (uint16_t)((ft[3] >> 16) & 0xFFFFu);
            dma_put_be16(buf + 12, frame_flags_merge(fmt, fflags));
        }

        if (p->deskew) {
            dma_deskew(buf + FRAME_HDR_LEN, (const uint16_t *)(const void *)base,
                       p->frame_w, p->frame_h, p->frame_w,
                       p->bufsize / 2, p->deskew_prev);
        } else if (p->geo_rows > 0 && p->geo_srcw > 0) {
            dma_geo_resample(buf + FRAME_HDR_LEN, base, p->frame_w, p->frame_h,
                             p->geo_stride, p->geo_rows, p->geo_srcw,
                             p->geo_xoff, p->bufsize);
        } else {
            dma_copy_pixels(buf + FRAME_HDR_LEN, base, p->payload_bytes, p->swap_mode);
        }

        if (p->rot != 0)
            dma_rot_rows(buf + FRAME_HDR_LEN, p->frame_w, p->frame_h, p->rot);

        if (p->track && p->prof_cur && p->prof_prv) {
            dma_prof_cols(buf + FRAME_HDR_LEN, p->frame_w, p->frame_h, p->prof_cur);
            if (p->have_prof) {
                int dx = dma_find_dx(p->prof_cur, p->prof_prv, p->frame_w);
                p->acc_dx -= dx;
                p->acc_dx %= p->frame_w;
                if (p->acc_dx < 0) p->acc_dx += p->frame_w;
            }
            memcpy(p->prof_prv, p->prof_cur, (size_t)p->frame_w * sizeof(uint32_t));
            p->have_prof = 1;
            if (p->acc_dx != 0)
                dma_rot_rows(buf + FRAME_HDR_LEN, p->frame_w, p->frame_h, p->acc_dx);
        }

        {
            uint32_t ft2[16];
            (void)dma_footer_fetch(base, p->payload_bytes, ft2);
            if (ft2[1] != seq) {
                p->tears++;
                (void)ioctl(p->dma_fd, ADAS_DMA_SYNC_DEV, &a);
                return 0;
            }
        }

        crc = frame_crc16_ccitt(buf, (size_t)FRAME_HDR_LEN + p->payload_bytes);
        dma_put_be16(buf + FRAME_HDR_LEN + p->payload_bytes, crc);

        p->last_seq[i] = seq;
        p->frames++;
        (void)ioctl(p->dma_fd, ADAS_DMA_SYNC_DEV, &a);
        return (int)need;
    }

    usleep(500);
    return 0;
}

static void dma_close(arm_transport_t *self)
{
    dma_priv_t *p = (dma_priv_t *)self->priv;
    if (!p) return;
    if (p->tears)
        fprintf(stderr, "[dma] 免撕裂校验：丢弃了 %llu 帧（拷贝过程中被 FPGA 覆盖）\n",
                (unsigned long long)p->tears);
    if (p->bar && p->bar != MAP_FAILED) munmap((void *)p->bar, p->bar_len);
    if (p->bar_fd >= 0) close(p->bar_fd);
    if (p->bufs && p->bufs != MAP_FAILED)
        munmap(p->bufs, (size_t)p->nbufs * p->bufsize);
    if (p->dma_fd >= 0) close(p->dma_fd);
    if (p->frames)
        fprintf(stderr, "[dma] 共取到 %llu 帧\n", (unsigned long long)p->frames);
}

arm_transport_t *arm_transport_dma_new(const char *dev_path, const char *bar_path,
                                       int frame_w, int frame_h,
                                       frame_pixfmt_t fmt)
{
    arm_transport_t *t;
    dma_priv_t *p;
    int bpp;

    if (frame_w <= 0 || frame_h <= 0) return NULL;
    bpp = frame_bytes_per_px(fmt);
    if (bpp <= 0) return NULL;

    t = (arm_transport_t *)calloc(1, sizeof(*t));
    p = (dma_priv_t *)calloc(1, sizeof(*p));
    if (!t || !p) { free(t); free(p); return NULL; }

    p->dev_path      = dev_path ? dev_path : "/dev/adas_dma";
    p->bar_path      = bar_path;
    p->frame_w       = frame_w;
    p->frame_h       = frame_h;
    p->fmt           = fmt;
    p->payload_bytes = (uint32_t)(frame_w * frame_h * bpp);
    p->dma_fd        = -1;
    p->bar_fd        = -1;
    p->swap_mode     = 0;

    t->name  = "pcie-dma";
    t->open  = dma_open;
    t->read  = dma_read;
    t->close = dma_close;
    t->priv  = p;
    return t;
}

typedef struct {
    const char *resource_path;
    size_t window;
    volatile uint32_t *doorbell;
    void *bar;
    int fd;
} pcie_priv_t;

static int pcie_open(arm_transport_t *self) {
    pcie_priv_t *p = (pcie_priv_t *)self->priv;
    p->fd = open(p->resource_path, O_RDWR | O_SYNC);
    if (p->fd < 0) { perror("open PCIe resource"); return -1; }
    p->bar = mmap(NULL, p->window, PROT_READ | PROT_WRITE, MAP_SHARED, p->fd, 0);
    if (p->bar == MAP_FAILED) { perror("mmap BAR"); p->bar = NULL; return -2; }
    return 0;
}

static int pcie_read(arm_transport_t *self, uint8_t *buf, size_t len) {
    pcie_priv_t *p = (pcie_priv_t *)self->priv;
    if (!p->bar) return -1;
    if (len > p->window) return -1;
    memcpy(buf, p->bar, len);
    return (int)len;
}

static void pcie_close(arm_transport_t *self) {
    pcie_priv_t *p = (pcie_priv_t *)self->priv;
    if (p->bar) munmap(p->bar, p->window);
    if (p->fd >= 0) close(p->fd);
    p->bar = NULL; p->fd = -1;
}

arm_transport_t *arm_transport_pcie_new(const char *resource_path, size_t window,
                                        volatile uint32_t *doorbell_reg) {
    arm_transport_t *t = (arm_transport_t *)calloc(1, sizeof(*t));
    pcie_priv_t *p = (pcie_priv_t *)calloc(1, sizeof(*p));
    if (!t || !p) { free(t); free(p); return NULL; }
    p->resource_path = resource_path; p->window = window;
    p->doorbell = doorbell_reg; p->bar = NULL; p->fd = -1;
    t->name = "pcie-shmem";
    t->open = pcie_open; t->read = pcie_read; t->close = pcie_close; t->priv = p;
    return t;
}


#define BUF_SLOTS 3

typedef struct {
    uint8_t *data;
    uint16_t seq;
    int      w, h;
    int      ready;
    int      busy;
    double   t_recv;
} slot_t;

struct arm_pipeline {
    arm_transport_t *tr;
    arm_infer_cb     cb;
    void            *cb_ctx;
    int              require_crc;

    size_t           frame_len;
    int              frame_w, frame_h;
    frame_pixfmt_t   frame_fmt;

    arm_ctrl_cb      ctrl_cb;
    void            *ctrl_ctx;
    frame_ctrl_t     last_ctrl;
    int              has_last_ctrl;


    slot_t   slots[BUF_SLOTS];
    int      seen_first;
    int      head;
    int      tail;
    int      count;

    pthread_t        rx_tid, infer_tid;
    pthread_mutex_t  mtx;
    pthread_cond_t   cv_data;
    pthread_cond_t   cv_space;
    atomic_int       running;

    arm_pipeline_stats_t st;
    double infer_ms_sum;
    uint64_t infer_n;
};

static void *rx_thread(void *arg) {
    arm_pipeline_t *pl = (arm_pipeline_t *)arg;
    uint8_t *frame = (uint8_t *)malloc(pl->frame_len);
    if (!frame) return NULL;

    while (atomic_load(&pl->running)) {
        double t0 = now_ms();
        int n = pl->tr->read(pl->tr, frame, pl->frame_len);
        double t1 = now_ms();
        if (n < 0) break;
        if (n == 0) continue;
        if ((size_t)n != pl->frame_len) continue;

        pthread_mutex_lock(&pl->mtx);
        pl->st.bytes_read += (uint64_t)n;
        double dt = t1 - t0;
        pl->st.recv_ms_avg = pl->st.recv_ms_avg * 0.9 + dt * 0.1;
        if (dt > pl->st.recv_ms_max) pl->st.recv_ms_max = dt;

        frame_info_t fi;
        frame_status_t s = frame_validate_ex(frame, pl->frame_len, pl->require_crc, &fi);
        uint16_t seq = fi.seq;
        switch (s) {
            case FRAME_OK:       pl->st.frames_ok++; break;
            case FRAME_ERR_MAGIC:pl->st.err_magic++; break;
            case FRAME_ERR_SIZE: pl->st.err_size++;  break;
            case FRAME_ERR_CRC:  pl->st.err_crc++;   break;
        }
        if (s == FRAME_OK) {
            if (pl->ctrl_cb) {
                frame_ctrl_t c = fi.ctrl;
                if (!pl->has_last_ctrl || !frame_ctrl_equal(&c, &pl->last_ctrl)) {
                    pl->last_ctrl = c;
                    pl->has_last_ctrl = 1;
                    pl->ctrl_cb(&c, pl->ctrl_ctx);
                }
            }
            if (pl->seen_first) {
                int16_t diff = (int16_t)(seq - pl->st.last_seq);
                if (diff > 1)       pl->st.frames_lost += (uint64_t)(diff - 1);
                else if (diff <= 0) pl->st.frames_ooo++;
            } else {
                pl->seen_first = 1;
            }
            pl->st.last_seq = seq;
        }
        pthread_mutex_unlock(&pl->mtx);

        if (s != FRAME_OK) continue;

        pthread_mutex_lock(&pl->mtx);
        if (pl->count == BUF_SLOTS) {
            pl->tail = (pl->tail + 1) % BUF_SLOTS;
            pl->count--;
            pl->st.frames_dropped++;
        }
        for (int guard = 0; guard < BUF_SLOTS && pl->slots[pl->head].busy; ++guard) {
            pl->head = (pl->head + 1) % BUF_SLOTS;
            if (pl->count > 0) { pl->tail = pl->head; pl->count--; pl->st.frames_dropped++; }
        }
        slot_t *sl = &pl->slots[pl->head];
        {
            uint8_t *tmp = sl->data;
            sl->data = frame;
            frame = tmp;
        }
        sl->seq = seq;
        sl->w = fi.width;
        sl->h = fi.height;
        sl->t_recv = t1;
        sl->ready = 1;
        pl->head = (pl->head + 1) % BUF_SLOTS;
        pl->count++;
        pthread_cond_signal(&pl->cv_data);
        pthread_mutex_unlock(&pl->mtx);
    }
    free(frame);
    return NULL;
}

static void *infer_thread(void *arg) {
    arm_pipeline_t *pl = (arm_pipeline_t *)arg;
    while (1) {
        pthread_mutex_lock(&pl->mtx);
        while (pl->count == 0) {
            if (!atomic_load(&pl->running)) {
                pthread_mutex_unlock(&pl->mtx);
                return NULL;
            }
            pthread_cond_wait(&pl->cv_data, &pl->mtx);
        }
        slot_t *sl = &pl->slots[pl->tail];
        uint16_t seq = sl->seq;
        int fw = sl->w, fh = sl->h;
        sl->busy = 1;
        const uint8_t *gray = frame_payload(sl->data);
        pl->tail = (pl->tail + 1) % BUF_SLOTS;
        pl->count--;
        pthread_cond_signal(&pl->cv_space);
        pthread_mutex_unlock(&pl->mtx);

        if (pl->cb) {
            double t0 = now_ms();
            pl->cb(gray, fw, fh, seq, pl->cb_ctx);
            pthread_mutex_lock(&pl->mtx);
            sl->busy = 0;
            pthread_mutex_unlock(&pl->mtx);
            double dt = now_ms() - t0;
            pthread_mutex_lock(&pl->mtx);
            pl->infer_ms_sum += dt;
            pl->infer_n++;
            pl->st.infer_ms_avg = pl->infer_ms_sum / (double)pl->infer_n;
            if (dt > pl->st.infer_ms_max) pl->st.infer_ms_max = dt;
            pthread_mutex_unlock(&pl->mtx);
        }
    }
    return NULL;
}

arm_pipeline_t *arm_pipeline_new_ex(arm_transport_t *transport,
                                    arm_infer_cb infer_cb, void *infer_ctx,
                                    int require_crc,
                                    int frame_w, int frame_h,
                                    frame_pixfmt_t frame_fmt) {
    if (!transport) return NULL;
    if (frame_w <= 0 || frame_h <= 0) return NULL;
    const int bpp = frame_bytes_per_px(frame_fmt);
    if (bpp == 0) return NULL;

    arm_pipeline_t *pl = (arm_pipeline_t *)calloc(1, sizeof(*pl));
    if (!pl) return NULL;
    pl->tr = transport;
    pl->cb = infer_cb;
    pl->cb_ctx = infer_ctx;
    pl->require_crc = require_crc;
    pl->frame_w = frame_w;
    pl->frame_h = frame_h;
    pl->frame_fmt = frame_fmt;
    pl->frame_len = frame_total_len_of(frame_w, frame_h, frame_fmt);
    for (int i = 0; i < BUF_SLOTS; ++i) {
        pl->slots[i].data = (uint8_t *)malloc(pl->frame_len);
        if (!pl->slots[i].data) { arm_pipeline_free(pl); return NULL; }
    }
    pthread_mutex_init(&pl->mtx, NULL);
    pthread_cond_init(&pl->cv_data, NULL);
    pthread_cond_init(&pl->cv_space, NULL);
    atomic_store(&pl->running, 0);
    return pl;
}

void arm_pipeline_set_ctrl_cb(arm_pipeline_t *pl, arm_ctrl_cb cb, void *ctx) {
    if (!pl) return;
    pl->ctrl_cb  = cb;
    pl->ctrl_ctx = ctx;
}

arm_pipeline_t *arm_pipeline_new(arm_transport_t *transport,
                                 arm_infer_cb infer_cb, void *infer_ctx,
                                 int require_crc) {
    return arm_pipeline_new_ex(transport, infer_cb, infer_ctx, require_crc,
                               FRAME_W, FRAME_H, FRAME_PIX_GRAY8);
}

int arm_pipeline_start(arm_pipeline_t *pl) {
    if (!pl) return -1;
    if (pl->tr->open && pl->tr->open(pl->tr) != 0) return -2;
    atomic_store(&pl->running, 1);
    if (pthread_create(&pl->rx_tid, NULL, rx_thread, pl) != 0) {
        atomic_store(&pl->running, 0);
        return -3;
    }
    if (pthread_create(&pl->infer_tid, NULL, infer_thread, pl) != 0) {
        atomic_store(&pl->running, 0);
        pthread_join(pl->rx_tid, NULL);
        return -4;
    }
    return 0;
}

void arm_pipeline_stop(arm_pipeline_t *pl) {
    if (!pl) return;
    atomic_store(&pl->running, 0);
    pthread_mutex_lock(&pl->mtx);
    pthread_cond_broadcast(&pl->cv_data);
    pthread_cond_broadcast(&pl->cv_space);
    pthread_mutex_unlock(&pl->mtx);
    pthread_join(pl->rx_tid, NULL);
    pthread_join(pl->infer_tid, NULL);
    if (pl->tr->close) pl->tr->close(pl->tr);
}

void arm_pipeline_free(arm_pipeline_t *pl) {
    if (!pl) return;
    for (int i = 0; i < BUF_SLOTS; ++i) free(pl->slots[i].data);
    pthread_mutex_destroy(&pl->mtx);
    pthread_cond_destroy(&pl->cv_data);
    pthread_cond_destroy(&pl->cv_space);
    if (pl->tr) {
        free(pl->tr->priv);
        free(pl->tr);
    }
    free(pl);
}

void arm_pipeline_stats(const arm_pipeline_t *pl, arm_pipeline_stats_t *out) {
    if (!pl || !out) return;
    pthread_mutex_lock((pthread_mutex_t *)&pl->mtx);
    *out = pl->st;
    pthread_mutex_unlock((pthread_mutex_t *)&pl->mtx);
}

void arm_pipeline_stats_print(const arm_pipeline_t *pl) {
    arm_pipeline_stats_t s;
    memset(&s, 0, sizeof(s));
    arm_pipeline_stats(pl, &s);
    uint64_t total = s.frames_ok + s.err_magic + s.err_size + s.err_crc;
    printf("[pipeline %s] 帧 %llu 好 / %llu 坏 (magic%llu size%llu crc%llu) | "
           "丢帧 %llu 乱序 %llu 丢弃 %llu | 接收 avg %.1fms max %.1fms | 推理 avg %.1fms max %.1fms\n",
           pl->tr->name,
           (unsigned long long)s.frames_ok, (unsigned long long)(total - s.frames_ok),
           (unsigned long long)s.err_magic, (unsigned long long)s.err_size,
           (unsigned long long)s.err_crc, (unsigned long long)s.frames_lost,
           (unsigned long long)s.frames_ooo, (unsigned long long)s.frames_dropped,
           s.recv_ms_avg, s.recv_ms_max, s.infer_ms_avg, s.infer_ms_max);
}
