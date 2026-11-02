#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>

#include "frame_parse.h"
#include "frame_proto.h"
#include "infer_rknn.h"
#include "decision.h"
#include "uart_tx.h"
#include "display.h"

_Static_assert((int)INFER_PIX_GRAY8  == (int)FRAME_PIX_GRAY8  &&
               (int)INFER_PIX_RGB565 == (int)FRAME_PIX_RGB565 &&
               (int)INFER_PIX_BGR565 == (int)FRAME_PIX_BGR565 &&
               (int)INFER_PIX_RGB888 == (int)FRAME_PIX_RGB888 &&
               (int)INFER_PIX_BGR888 == (int)FRAME_PIX_BGR888,
               "infer_pixfmt_t 与 frame_pixfmt_t 必须一致");

#define PRE_RING 3

typedef struct {
    pthread_mutex_t mtx;
    pthread_cond_t  cv_data, cv_space;
    int      head, tail, count;
    int      stop;
    int      started;
    uint8_t *buf[PRE_RING];
    infer_lb_t lb[PRE_RING];
    double   t_pre[PRE_RING];
    unsigned seq[PRE_RING];
    int      fw[PRE_RING], fh[PRE_RING];
    int      dslot[PRE_RING];
    pthread_t tid;
    double   cb_ms_total, wk_ms_total;
} pre_stage_t;

#define DSP_RING   4
#define DSP_MAXBOX 256

typedef struct {
    pthread_mutex_t mtx;
    pthread_cond_t  cv_frame, cv_shot, cv_empty;
    int      state[DSP_RING];
    uint8_t *buf[DSP_RING];
    pp_box_t box[DSP_RING][DSP_MAXBOX];
    int      nbox[DSP_RING];
    char     osd[DSP_RING][112];
    int      src_w, src_h;
    int      p_idx, c_idx;
    int      stop;

    display_t *disp;
    pthread_t  tid;
    int        started;
    double t_sum; int n_renders; double t_max;
} disp_stage_t;

static int dsp_reserve(disp_stage_t *s) {
    pthread_mutex_lock(&s->mtx);
    while (s->state[s->p_idx] != 0 && !s->stop)
        pthread_cond_wait(&s->cv_empty, &s->mtx);
    if (s->stop) { pthread_mutex_unlock(&s->mtx); return -1; }
    int i = s->p_idx;
    pthread_mutex_unlock(&s->mtx);
    return i;
}
static void dsp_put_frame(disp_stage_t *s, int i) {
    pthread_mutex_lock(&s->mtx);
    s->state[i] = 1;
    s->p_idx = (s->p_idx + 1) % DSP_RING;
    pthread_cond_broadcast(&s->cv_frame);
    pthread_mutex_unlock(&s->mtx);
}
static void dsp_put_boxes(disp_stage_t *s, int i, const pp_box_t *b, int n, const char *osd) {
    pthread_mutex_lock(&s->mtx);
    int k = n > DSP_MAXBOX ? DSP_MAXBOX : n;
    if (k > 0) memcpy(s->box[i], b, (size_t)k * sizeof(pp_box_t));
    s->nbox[i] = k;
    if (osd) snprintf(s->osd[i], sizeof(s->osd[i]), "%s", osd);
    else     s->osd[i][0] = 0;
    s->state[i] = 2;
    pthread_cond_broadcast(&s->cv_shot);
    pthread_mutex_unlock(&s->mtx);
}

static double now_ms_d2(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static void *dsp_thread(void *arg) {
    disp_stage_t *s = (disp_stage_t *)arg;
    for (;;) {
        pthread_mutex_lock(&s->mtx);
        while (s->state[s->c_idx] != 2 && !s->stop)
            pthread_cond_wait(&s->cv_shot, &s->mtx);
        if (s->stop && s->state[s->c_idx] != 2) { pthread_mutex_unlock(&s->mtx); return NULL; }
        const int i = s->c_idx;
        pthread_mutex_unlock(&s->mtx);

        double t0 = now_ms_d2();
        display_show(s->disp, s->buf[i], s->box[i], s->nbox[i], s->osd[i]);
        double dt = now_ms_d2() - t0;

        pthread_mutex_lock(&s->mtx);
        s->t_sum += dt; s->n_renders++;
        if (dt > s->t_max) s->t_max = dt;
        s->state[i] = 0;
        s->c_idx = (s->c_idx + 1) % DSP_RING;
        pthread_cond_broadcast(&s->cv_empty);
        pthread_mutex_unlock(&s->mtx);
    }
}

static int pre_stage_reserve(pre_stage_t *s) {
    pthread_mutex_lock(&s->mtx);
    while (s->count >= PRE_RING && !s->stop)
        pthread_cond_wait(&s->cv_space, &s->mtx);
    if (s->stop) { pthread_mutex_unlock(&s->mtx); return -1; }
    int slot = s->head;
    pthread_mutex_unlock(&s->mtx);
    return slot;
}

static void pre_stage_commit(pre_stage_t *s) {
    pthread_mutex_lock(&s->mtx);
    s->head = (s->head + 1) % PRE_RING;
    s->count++;
    pthread_cond_signal(&s->cv_data);
    pthread_mutex_unlock(&s->mtx);
}

typedef struct {
    double t;
    int    field;
    int    value;
} key_ev_t;

enum { KEY_LANE = 0, KEY_MOTION, KEY_LLINE, KEY_RLINE };

typedef struct {
    infer_pixfmt_t fmt;
    int            src_w, src_h;
    pp_box_t      *boxes;
    int            max_boxes;
    dec_cfg_t      dec_cfg;
    uart_tx_t     *uart;
    dec_input_ext_t ext;

    int            quad;
    uint8_t       *qbuf;
    pp_box_t      *qtmp;
    int            no_infer;
    int            size_warned;
    unsigned long  n_frames;
    unsigned long  n_boxes_total;
    int            verbose;
    const char    *dump_path;
    int            dump_done;
    double         t_first;
    double         t_first_frame, t_last_frame;

    key_ev_t      *keys;
    int            n_keys, key_idx, keys_active;
    unsigned       key_mask;
    double         key_fps;
    pre_stage_t    pre;

    disp_stage_t   dsp;
} adas_ctx_t;

static void adas_ctrl_cb(const frame_ctrl_t *c, void *ctx) {
    adas_ctx_t *a = (adas_ctx_t *)ctx;
    if (a->keys_active) {
        if (c->light) a->ext.light = (light_state_t)c->light;
        if (!(a->key_mask & (1u << KEY_LLINE)))
            a->ext.left_line  = (lane_line_t)c->lline;
        if (!(a->key_mask & (1u << KEY_RLINE)))
            a->ext.right_line = (lane_line_t)c->rline;
        if (!(a->key_mask & (1u << KEY_LANE)) && c->lane)
            a->ext.lane = (lane_state_t)c->lane;
        if (!(a->key_mask & (1u << KEY_MOTION)) && c->motion)
            a->ext.motion = (motion_state_t)c->motion;
        return;
    }
    a->ext.left_line  = (lane_line_t)c->lline;
    a->ext.right_line = (lane_line_t)c->rline;
    if (c->lane)   a->ext.lane   = (lane_state_t)c->lane;
    if (c->motion) a->ext.motion = (motion_state_t)c->motion;
    if (c->light)  a->ext.light  = (light_state_t)c->light;
    fprintf(stderr, "[adas] FPGA 状态: 左线=%u 右线=%u | FPGA 另给: 车道=%u 行驶=%u 灯色=%u\n",
            (unsigned)c->lline, (unsigned)c->rline,
            (unsigned)c->lane, (unsigned)c->motion, (unsigned)c->light);
}

static volatile int g_stop = 0;
static void on_sigint(int s) { (void)s; g_stop = 1; }

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static int key_field_id(const char *s) {
    if (!strcmp(s, "lane"))   return KEY_LANE;
    if (!strcmp(s, "motion")) return KEY_MOTION;
    if (!strcmp(s, "lline") || !strcmp(s, "left_line"))  return KEY_LLINE;
    if (!strcmp(s, "rline") || !strcmp(s, "right_line")) return KEY_RLINE;
    return -1;
}

static int key_field_value(int field, const char *s) {
    if (field == KEY_LANE) {
        if (!strcmp(s, "left"))     return LANE_LEFT;
        if (!strcmp(s, "straight")) return LANE_STRAIGHT;
        if (!strcmp(s, "right"))    return LANE_RIGHT;
        if (!strcmp(s, "unknown"))  return LANE_UNKNOWN;
    } else if (field == KEY_MOTION) {
        if (!strcmp(s, "running"))  return MOTION_RUNNING;
        if (!strcmp(s, "stopped"))  return MOTION_STOPPED;
        if (!strcmp(s, "unknown"))  return MOTION_UNKNOWN;
    } else if (field == KEY_LLINE || field == KEY_RLINE) {
        if (!strcmp(s, "solid"))    return LINE_SOLID;
        if (!strcmp(s, "dashed"))   return LINE_DASHED;
        if (!strcmp(s, "unknown"))  return LINE_UNKNOWN;
    }
    return -1;
}

static int key_cmp(const void *pa, const void *pb) {
    const key_ev_t *a = (const key_ev_t *)pa;
    const key_ev_t *b = (const key_ev_t *)pb;
    if (a->t < b->t) return -1;
    if (a->t > b->t) return 1;
    return a->field - b->field;
}

static int key_load(adas_ctx_t *a, const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "[keys] 打不开 %s: %s\n", path, strerror(errno));
        return -1;
    }
    char line[256];
    int cap = 0;
    while (fgets(line, sizeof(line), f)) {
        char *p = line;
        while (*p == ' ' || *p == '\t') ++p;
        if (*p == '#' || *p == '\0' || *p == '\n') continue;
        double t_ms;
        char fld[32], val[32];
        if (sscanf(p, "%lf %31s %31s", &t_ms, fld, val) != 3) {
            fprintf(stderr, "[keys] 跳过无法解析的行: %s", line);
            continue;
        }
        int field = key_field_id(fld);
        int value = (field >= 0) ? key_field_value(field, val) : -1;
        if (field < 0 || value < 0) {
            fprintf(stderr, "[keys] 跳过未知字段/值: %s %s\n", fld, val);
            continue;
        }
        if (a->n_keys >= cap) {
            cap = cap ? cap * 2 : 32;
            key_ev_t *n = (key_ev_t *)realloc(a->keys, (size_t)cap * sizeof(*n));
            if (!n) { fclose(f); fprintf(stderr, "[keys] 内存不足\n"); return -1; }
            a->keys = n;
        }
        a->keys[a->n_keys].t = t_ms / 1000.0;
        a->keys[a->n_keys].field = field;
        a->keys[a->n_keys].value = value;
        a->n_keys++;
        a->key_mask |= (1u << field);
    }
    fclose(f);
    if (a->n_keys > 0) {
        qsort(a->keys, (size_t)a->n_keys, sizeof(*a->keys), key_cmp);
        a->keys_active = 1;
        printf("[keys] 已加载 %d 条时间线事件（%s），按 seq/%.1ffps 推进\n",
               a->n_keys, path, a->key_fps);
    } else {
        printf("[keys] %s 没有有效事件\n", path);
    }
    return 0;
}

static void key_apply(adas_ctx_t *a, unsigned seq) {
    if (!a->keys_active) return;
    double t = (double)seq / (a->key_fps > 0.1 ? a->key_fps : 30.0);
    while (a->key_idx < a->n_keys && a->keys[a->key_idx].t <= t + 1e-9) {
        const key_ev_t *e = &a->keys[a->key_idx++];
        switch (e->field) {
            case KEY_LANE:   a->ext.lane   = (lane_state_t)e->value;   break;
            case KEY_MOTION: a->ext.motion = (motion_state_t)e->value; break;
            case KEY_LLINE:  a->ext.left_line  = (lane_line_t)e->value; break;
            case KEY_RLINE:  a->ext.right_line = (lane_line_t)e->value; break;
            default: break;
        }
        if (a->verbose)
            fprintf(stderr, "[keys] t=%.2fs field=%d value=%d\n", t, e->field, e->value);
    }
}

static void adas_process_frame(adas_ctx_t *c, int n, int w, int h, unsigned seq) {
    if (c->keys_active) key_apply(c, seq);
    if ((w != c->src_w || h != c->src_h) && !c->size_warned) {
        fprintf(stderr, "[adas] ⚠️ 帧头尺寸 %d×%d 与配置 %d×%d 不一致！\n",
                w, h, c->src_w, c->src_h);
        c->size_warned = 1;
    }

    dec_input_ext_t ext = c->ext;
    ext.ts_ms = (uint32_t)(now_s() * 1000.0);
    dec_output_t dec;
    if (dec_run(c->boxes, n, &ext, &c->dec_cfg, &dec) != 0) return;

    if (c->uart) {
        int sent = uart_tx_send(c->uart, seq, &dec, c->boxes, n, w, h);
        if (sent < 0 && c->verbose)
            fprintf(stderr, "[adas] uart_tx_send 失败 %d\n", sent);
    }

    c->n_frames++;
    c->n_boxes_total += (unsigned long)n;

    if (c->verbose) {
        int cc[5] = {0, 0, 0, 0, 0};
        for (int k = 0; k < n; ++k) {
            int cl = c->boxes[k].cls;
            if (cl >= 0 && cl < 5) cc[cl]++;
        }
        fprintf(stderr,
                "[adas] seq=%-6u 框=%-3d p=%d c=%d tl_r=%d tl_g=%d tl_y=%d cmd=%d alarms=0x%02x\n",
                seq, n, cc[0], cc[1], cc[2], cc[3], cc[4], (int)dec.cmd, dec.alarms);
    }
    if (c->n_frames == 1) c->t_first_frame = now_s();
    c->t_last_frame = now_s();
}

static void *adas_worker(void *arg) {
    adas_ctx_t *c = (adas_ctx_t *)arg;
    pre_stage_t *s = &c->pre;
    for (;;) {
        pthread_mutex_lock(&s->mtx);
        while (s->count == 0 && !s->stop)
            pthread_cond_wait(&s->cv_data, &s->mtx);
        if (s->count == 0 && s->stop) {
            pthread_mutex_unlock(&s->mtx);
            return NULL;
        }
        const int     slot = s->tail;
        const unsigned seq = s->seq[slot];
        const int     fw   = s->fw[slot], fh = s->fh[slot];
        const infer_lb_t lb = s->lb[slot];
        const double  t_pre = s->t_pre[slot];
        pthread_mutex_unlock(&s->mtx);

        int n = 0;
        const double t_b0 = now_s();
        int rc = infer_run_stage(&lb, s->buf[slot], c->boxes, c->max_boxes,
                                 &n, 1, t_pre);
        if (rc == 0) {
            adas_process_frame(c, n, fw, fh, seq);
            if (c->dsp.disp && c->pre.dslot[slot] >= 0) {
                char osd[112];
                int cc[5] = {0, 0, 0, 0, 0};
                for (int k = 0; k < n; ++k) {
                    int cl = c->boxes[k].cls;
                    if (cl >= 0 && cl < 5) cc[cl]++;
                }
                infer_stat_t ds; infer_get_stat(&ds);
                snprintf(osd, sizeof(osd),
                         "FPS %.1f  BOX %d  P%d C%d R%d G%d Y%d  %.1fMS",
                         ds.avg_fps, n, cc[0], cc[1], cc[2], cc[3], cc[4],
                         ds.avg_total);
                dsp_put_boxes(&c->dsp, c->pre.dslot[slot], c->boxes, n, osd);
            }
        } else if (c->verbose) {
            fprintf(stderr, "[adas] worker 推理失败 rc=%d (seq=%u)\n", rc, seq);
        }

        pthread_mutex_lock(&s->mtx);
        s->wk_ms_total += (now_s() - t_b0) * 1000.0;
        s->count--;
        s->tail = (s->tail + 1) % PRE_RING;
        pthread_cond_broadcast(&s->cv_space);
        pthread_mutex_unlock(&s->mtx);
    }
}

static void adas_infer_cb(const uint8_t *pix, int w, int h,
                          uint16_t seq, void *ctx) {
    adas_ctx_t *c = (adas_ctx_t *)ctx;
    if (c->dump_path && !c->dump_done && pix && w > 0 && h > 0) {
        const int bpp = infer_bytes_per_px(c->fmt);
        FILE *fp = fopen(c->dump_path, "wb");
        if (fp) {
            size_t nb = (size_t)w * (size_t)h * (size_t)bpp;
            size_t wn = fwrite(pix, 1, nb, fp);
            fclose(fp);
            fprintf(stderr, "[dump] 第1帧 %dx%d bpp=%d -> %s（%zu/%zu 字节）\n",
                    w, h, bpp, c->dump_path, wn, nb);
        } else {
            fprintf(stderr, "[dump] 打不开 %s: %s\n", c->dump_path, strerror(errno));
        }
        c->dump_done = 1;
    }
    int n = 0;
    if (!c->no_infer) {
        if (c->quad == 2 && w >= 2 && h >= 2 && c->qbuf && c->qtmp) {
            const int bpp = infer_bytes_per_px(c->fmt);
            const int qw = w / 2, qh = h / 2;
            for (int qi = 0; qi < 4; ++qi) {
                const int qx = qi & 1, qy = qi >> 1;
                const int qn0 = c->max_boxes;
                int qn = 0;

                for (int r = 0; r < qh; ++r)
                    memcpy(c->qbuf + (size_t)r * qw * bpp,
                           pix + ((size_t)(qy * qh + r) * w + (size_t)qx * qw) * bpp,
                           (size_t)qw * bpp);

                int rc = infer_run_ex(c->qbuf, qw, qh, c->fmt,
                                      c->qtmp, qn0, &qn, qi == 0);
                if (rc != 0) {
                    if (c->verbose)
                        fprintf(stderr, "[adas] 象限%d 推理失败 rc=%d（seq=%u）\n",
                                qi, rc, seq);
                    continue;
                }

                int kept = 0;
                for (int k = 0; k < qn && n < c->max_boxes; ++k) {
                    pp_box_t b = c->qtmp[k];
                    b.x1 += qx * qw;  b.x2 += qx * qw;
                    b.y1 += qy * qh;  b.y2 += qy * qh;
                    c->boxes[n++] = b;
                    kept++;
                }
                if (c->verbose)
                    fprintf(stderr, "[adas] 象限%d(Q%d%d) 框=%d\n", qi, qy, qx, kept);
            }
        } else {
            int slot = pre_stage_reserve(&c->pre);
            if (slot < 0) return;
            const double t0 = now_s();
            if (infer_pre_stage(pix, w, h, c->fmt,
                                c->pre.buf[slot], &c->pre.lb[slot]) != 0) {
                if (c->verbose)
                    fprintf(stderr, "[adas] 预处理失败（seq=%u）\n", seq);
                return;
            }
            int dslot = -1;
            if (c->dsp.disp) {
                dslot = dsp_reserve(&c->dsp);
                if (dslot >= 0) {
                    const int bpp = infer_bytes_per_px(c->fmt);
                    memcpy(c->dsp.buf[dslot], pix,
                           (size_t)w * (size_t)h * (size_t)bpp);
                    dsp_put_frame(&c->dsp, dslot);
                }
            }
            c->pre.dslot[slot] = dslot;
            c->pre.t_pre[slot] = (now_s() - t0) * 1000.0;
            c->pre.seq[slot]   = seq;
            c->pre.fw[slot]    = w;
            c->pre.fh[slot]    = h;
            pre_stage_commit(&c->pre);
            pthread_mutex_lock(&c->pre.mtx);
            c->pre.cb_ms_total += (now_s() - t0) * 1000.0;
            pthread_mutex_unlock(&c->pre.mtx);
            return;
        }
    }
    adas_process_frame(c, n, w, h, seq);

    if (c->dsp.disp) {
        char osd[112];
        int cc[5] = {0, 0, 0, 0, 0};
        for (int k = 0; k < n; ++k) {
            int cl = c->boxes[k].cls;
            if (cl >= 0 && cl < 5) cc[cl]++;
        }
        infer_stat_t ds; infer_get_stat(&ds);
        snprintf(osd, sizeof(osd),
                 "FPS %.1f  BOX %d  P%d C%d R%d G%d Y%d  %.1fMS",
                 ds.avg_fps, n, cc[0], cc[1], cc[2], cc[3], cc[4], ds.avg_total);
        display_show(c->dsp.disp, pix, c->boxes, n, osd);
    }
}

static void usage(const char *p) {
    fprintf(stderr,
        "用法: %s --model <best.rknn> --transport <spec> [选项]\n"
        "\n"
        "  --model PATH       .rknn 模型路径（必填）\n"
        "  --transport SPEC   dma:<BAR资源路径>[:<设备>] | spi:/dev/spidevX.Y |\n"
        "                     file:frames.bin | pcie:/dev/xxx\n"
        "                     raw:/dev/stdin  ← 裸像素流，配 ffmpeg 管道\n"
        "                     dma: 例 = dma:/sys/bus/pci/devices/0000:01:00.0/resource0:/dev/adas_dma\n"
        "                     最后一个冒号是 BAR 与设备的分隔符（BDF 里也有冒号）\n"
        "  --pixfmt FMT       gray8 | rgb565 | bgr565 | rgb888 | bgr888  (默认 rgb565)\n"
        "  --src WxH          输入帧尺寸，必须与 FPGA 一致            (默认 960x720)\n"
        "  --uart SPEC        /dev/ttyS3 或 file:/tmp/out.bin    (默认不发)\n"
        "  --keys FILE        场景按键时间线（t_ms lane/motion/lline/rline）\n"
        "                     也可用 ADAS_KEY_FILE；fps 用 ADAS_SCENARIO_FPS（默认30）\n"
        "  --baud N           串口波特率                        (默认 460800)\n"
        "  --conf F           置信度阈值                          (默认 0.25)\n"
        "  --iou  F           NMS IoU 阈值                        (默认 0.45)\n"
        "  --frames N         处理 N 帧后退出（0=不限）\n"
        "  --no-crc           跳过 CRC 校验（早期联调用）\n"
        "  --no-infer         跳过 NPU 推理，只跑传输/决策/回传（无板子调链路用）\n"
        "  --log-every N      每 N 帧打印一次统计             (默认 30)\n"
        "  --display PATH     把带框画面画到 framebuffer，如 /dev/fb0（默认关）\n"
        "                     ★ 用之前先 sudo systemctl stop lightdm，否则被 X 覆盖\n"
        "  --quad N           四路拼接模式：N=2 表示 2x2 画面（每帧跑 4 次推理），\n"
        "                     象限布局 Q0 左上 / Q1 右上 / Q2 左下 / Q3 右下\n"
        "                     0 = 关闭（默认，整帧一次推理）\n"
        "  --dump-frame PATH  把收到的**第一帧原始像素**存成文件（联调用）\n"
        "                     用来在 PC 上还原成 PNG，判断字节序/行宽/错行\n"
        "  --verbose          每帧打印详情\n"
        "\n"
        "  ⚠️ --pixfmt gray8 会让红/黄/绿灯退化成同一类（三通道相同），\n"
        "     只用于链路联调；正式演示必须用彩色。\n",
        p);
}

static infer_pixfmt_t parse_fmt(const char *s) {
    if (!strcmp(s, "gray8"))  return INFER_PIX_GRAY8;
    if (!strcmp(s, "rgb565")) return INFER_PIX_RGB565;
    if (!strcmp(s, "bgr565")) return INFER_PIX_BGR565;
    if (!strcmp(s, "rgb888")) return INFER_PIX_RGB888;
    if (!strcmp(s, "bgr888")) return INFER_PIX_BGR888;
    fprintf(stderr, "未知 pixfmt: %s\n", s);
    exit(2);
}

int main(int argc, char **argv) {
    const char *model = NULL, *transport = NULL, *uart_spec = NULL;
    const char *keys_spec = NULL;
    const char *fmt_s = "rgb565";
    int src_w = 960, src_h = 720, baud = 460800;
    float conf = 0.25f, iou = 0.45f;
    long max_frames = 0, log_every = 30;
    int quad = 0;
    const char *display_spec = NULL;
    const char *dump_spec = NULL;
    int require_crc = 1, verbose = 0, no_infer = 0;

    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        if      (!strcmp(a, "--model")     && i + 1 < argc) model = argv[++i];
        else if (!strcmp(a, "--transport") && i + 1 < argc) transport = argv[++i];
        else if (!strcmp(a, "--uart")      && i + 1 < argc) uart_spec = argv[++i];
        else if (!strcmp(a, "--keys")      && i + 1 < argc) keys_spec = argv[++i];
        else if (!strcmp(a, "--pixfmt")    && i + 1 < argc) fmt_s = argv[++i];
        else if (!strcmp(a, "--src")       && i + 1 < argc) sscanf(argv[++i], "%dx%d", &src_w, &src_h);
        else if (!strcmp(a, "--baud")      && i + 1 < argc) baud = atoi(argv[++i]);
        else if (!strcmp(a, "--conf")      && i + 1 < argc) conf = (float)atof(argv[++i]);
        else if (!strcmp(a, "--iou")       && i + 1 < argc) iou  = (float)atof(argv[++i]);
        else if (!strcmp(a, "--frames")    && i + 1 < argc) max_frames = atol(argv[++i]);
        else if (!strcmp(a, "--log-every") && i + 1 < argc) log_every = atol(argv[++i]);
        else if (!strcmp(a, "--quad")      && i + 1 < argc) quad = atoi(argv[++i]);
        else if (!strcmp(a, "--display")   && i + 1 < argc) display_spec = argv[++i];
        else if (!strcmp(a, "--no-crc"))   require_crc = 0;
        else if (!strcmp(a, "--no-infer")) no_infer = 1;
        else if (!strcmp(a, "--dump-frame") && i + 1 < argc) dump_spec = argv[++i];
        else if (!strcmp(a, "--verbose"))  verbose = 1;
        else if (!strcmp(a, "-h") || !strcmp(a, "--help")) { usage(argv[0]); return 0; }
        else { fprintf(stderr, "未知参数 %s\n", a); usage(argv[0]); return 2; }
    }
    if (!transport || (!model && !no_infer)) { usage(argv[0]); return 2; }

    printf("============================================================\n");
    printf(" ADAS 检测服务（模块 21~26 串联）\n");
    printf("------------------------------------------------------------\n");
    printf("   模型    : %s\n", model);
    printf("   链路    : %s\n", transport);
    printf("   像素格式: %s   %d×%d\n", fmt_s, src_w, src_h);
    printf("   UART    : %s\n", uart_spec ? uart_spec : "(不回传)");
    printf("   按键时间线: %s\n", keys_spec ? keys_spec : "(无)");
    printf("   阈值    : conf=%.2f iou=%.2f  CRC=%s\n", conf, iou, require_crc ? "开" : "关");
    printf("============================================================\n");

    if (parse_fmt(fmt_s) == INFER_PIX_GRAY8)
        printf("\n⚠️  灰度模式：红/黄/绿灯三通道相同，颜色分类会失效，仅用于联调\n\n");

    int rc = -2;
    if (!no_infer) {
        infer_cfg_t icfg = infer_default_cfg(model);
        icfg.conf_thres = conf;
        icfg.iou_thres  = iou;
        rc = infer_init(&icfg);
    }
    if (rc != 0 && !no_infer) {
        fprintf(stderr, "❌ infer_init 失败 rc=%d", rc);
        if (rc == -2) fprintf(stderr, "（本机未编译 RKNN 支持，需要交叉编译到板子；"
                                      "只想调链路可加 --no-infer）");
        fprintf(stderr, "\n");
        return 1;
    }
    if (rc != 0) printf("⚠️  --no-infer：跳过推理，只验证传输/决策/回传链路\n");

    uart_tx_t *uart = NULL;
    if (uart_spec) {
        if (!strncmp(uart_spec, "file:", 5)) uart = uart_tx_open_file(uart_spec + 5);
        else                                  uart = uart_tx_open_device(uart_spec, baud);
        if (!uart) { fprintf(stderr, "❌ 打不开 UART %s\n", uart_spec); infer_release(); return 1; }
    }

    if (!strncmp(transport, "raw:", 4) && require_crc) {
        require_crc = 0;
        fprintf(stderr, "[adas] raw 传输：已自动关闭 CRC 校验（省 ~20ms/帧）\n");
    }

    arm_transport_t *tr = NULL;
    if (!strncmp(transport, "spi:", 4))
        tr = arm_transport_spi_new(transport + 4, 50000000);
    else if (!strncmp(transport, "file:", 5))
        tr = arm_transport_file_new(transport + 5);
    else if (!strncmp(transport, "raw:", 4))
        tr = arm_transport_raw_new(transport + 4, src_w, src_h,
                                   (frame_pixfmt_t)parse_fmt(fmt_s));
    else if (!strncmp(transport, "dma:", 4))
        {
            char spec[512];
            char *barp = spec, *devp = NULL;
            strncpy(spec, transport + 4, sizeof(spec) - 1);
            spec[sizeof(spec) - 1] = '\0';
            {
                char *colon = strrchr(spec, ':');
                if (colon) { *colon = '\0'; devp = colon + 1; }
            }
            if (spec[0] == '\0' || !strcmp(spec, "none")) barp = NULL;
            tr = arm_transport_dma_new(devp && devp[0] ? devp : "/dev/adas_dma",
                                       barp, src_w, src_h,
                                       (frame_pixfmt_t)parse_fmt(fmt_s));
        }
    else if (!strncmp(transport, "pcie:", 5))
        tr = arm_transport_pcie_new(transport + 5, 0, NULL);
    if (!tr) { fprintf(stderr, "❌ 无法创建传输 %s\n", transport); if (uart) uart_tx_close(uart); infer_release(); return 1; }

    static adas_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.fmt       = parse_fmt(fmt_s);
    ctx.src_w     = src_w;
    ctx.src_h     = src_h;
    ctx.max_boxes = (quad == 2) ? 200 : 100;
    ctx.boxes     = (pp_box_t *)malloc(sizeof(pp_box_t) * ctx.max_boxes);

    ctx.quad = quad;
    if (quad == 2) {
        const int bpp = infer_bytes_per_px(ctx.fmt);
        const size_t qsz = (size_t)(src_w / 2) * (size_t)(src_h / 2) * (size_t)(bpp > 0 ? bpp : 1);
        ctx.qbuf = (uint8_t *)malloc(qsz);
        ctx.qtmp = (pp_box_t *)malloc(sizeof(pp_box_t) * ctx.max_boxes);
        if (!ctx.qbuf || !ctx.qtmp) {
            fprintf(stderr, "❌ 四路模式缓冲区分配失败（%zu 字节）\n", qsz);
            if (uart) uart_tx_close(uart);
            infer_release();
            return 1;
        }
        fprintf(stderr, "[adas] 四路拼接模式：%dx%d 整帧 -> 4 个 %dx%d 象限，象限缓冲 %zu 字节\n",
                src_w, src_h, src_w / 2, src_h / 2, qsz);
    }
    ctx.dec_cfg   = dec_default_cfg(src_w, src_h);
    ctx.uart      = uart;
    ctx.verbose   = verbose;
    ctx.no_infer  = no_infer;
    ctx.dump_path = dump_spec;
    ctx.t_first   = now_s();
    memset(&ctx.ext, 0, sizeof(ctx.ext));
    {
        const char *e;
        if ((e = getenv("ADAS_LANE")) && *e) {
            if      (!strcmp(e, "left"))     ctx.ext.lane = LANE_LEFT;
            else if (!strcmp(e, "straight")) ctx.ext.lane = LANE_STRAIGHT;
            else if (!strcmp(e, "right"))    ctx.ext.lane = LANE_RIGHT;
            else fprintf(stderr, "[adas] ADAS_LANE=%s 无法识别（left|straight|right）\n", e);
            if (ctx.ext.lane != LANE_UNKNOWN)
                fprintf(stderr, "[adas] 车道由 ARM 指定: %s\n", e);
        }
        if ((e = getenv("ADAS_MOTION")) && *e) {
            if      (!strcmp(e, "running")) ctx.ext.motion = MOTION_RUNNING;
            else if (!strcmp(e, "stopped")) ctx.ext.motion = MOTION_STOPPED;
            else fprintf(stderr, "[adas] ADAS_MOTION=%s 无法识别（running|stopped）\n", e);
            if (ctx.ext.motion != MOTION_UNKNOWN)
                fprintf(stderr, "[adas] 行驶状态由 ARM 指定: %s\n", e);
        }
    }

    ctx.key_fps = 30.0;
    {
        const char *e = getenv("ADAS_SCENARIO_FPS");
        if (e && *e) ctx.key_fps = atof(e);
    }
    {
        const char *kf = (keys_spec && *keys_spec) ? keys_spec : getenv("ADAS_KEY_FILE");
        if (kf && *kf) {
            if (key_load(&ctx, kf) != 0) {
                fprintf(stderr, "❌ 加载按键时间线失败: %s\n", kf);
                if (uart) uart_tx_close(uart);
                infer_release();
                return 2;
            }
        }
    }

    arm_pipeline_t *pl = arm_pipeline_new_ex(tr, adas_infer_cb, &ctx, require_crc,
                                             src_w, src_h,
                                             (frame_pixfmt_t)ctx.fmt);
    if (!pl) { fprintf(stderr, "❌ 创建流水线失败\n"); free(ctx.boxes); if (uart) uart_tx_close(uart); infer_release(); return 1; }

    arm_pipeline_set_ctrl_cb(pl, adas_ctrl_cb, &ctx);

    if (display_spec && *display_spec) {
        const char *p = display_spec;
        if (!strncmp(p, "fb:", 3)) p += 3;
        ctx.dsp.disp = display_open(p, src_w, src_h);
        if (!ctx.dsp.disp) {
            fprintf(stderr, "[adas] ⚠️  显示打开失败（%s），继续无显示运行\n",
                    display_last_error());
        }
    }

    const int use_stage = (!no_infer && ctx.quad != 2);
    if (use_stage) {
        int iw = 0, ih = 0;
        if (infer_input_dims(&iw, &ih) != 0 || iw <= 0 || ih <= 0) {
            fprintf(stderr, "❌ 取不到模型输入尺寸，无法建预处理缓冲\n");
            arm_pipeline_free(pl); free(ctx.boxes);
            if (uart) uart_tx_close(uart);
            infer_release();
            return 1;
        }
        const size_t bs = (size_t)iw * (size_t)ih * 3;
        memset(&ctx.pre, 0, sizeof(ctx.pre));
        for (int i = 0; i < PRE_RING; ++i) {
            ctx.pre.buf[i] = (uint8_t *)malloc(bs);
            if (!ctx.pre.buf[i]) {
                fprintf(stderr, "❌ 预处理缓冲分配失败（%zu 字节 × %d）\n", bs, PRE_RING);
                for (int j = 0; j < i; ++j) free(ctx.pre.buf[j]);
                arm_pipeline_free(pl); free(ctx.boxes);
                if (uart) uart_tx_close(uart);
                infer_release();
                return 1;
            }
        }
        pthread_mutex_init(&ctx.pre.mtx, NULL);
        pthread_cond_init(&ctx.pre.cv_data, NULL);
        pthread_cond_init(&ctx.pre.cv_space, NULL);
        if (pthread_create(&ctx.pre.tid, NULL, adas_worker, &ctx) != 0) {
            fprintf(stderr, "❌ 推理 worker 线程创建失败\n");
            for (int i = 0; i < PRE_RING; ++i) free(ctx.pre.buf[i]);
            memset(&ctx.pre, 0, sizeof(ctx.pre));
            arm_pipeline_free(pl); free(ctx.boxes);
            if (uart) uart_tx_close(uart);
            infer_release();
            return 1;
        }
        ctx.pre.started = 1;
        fprintf(stderr, "[adas] 两段流水已启用：预处理与 NPU 推理并行"
                        "（输入 %d×%d，双缓冲 %zu 字节×%d）\n", iw, ih, bs, PRE_RING);

        if (ctx.dsp.disp) {
            const size_t fsz = (size_t)src_w * (size_t)src_h *
                               (size_t)infer_bytes_per_px(ctx.fmt);
            int dio = 1;
            for (int i = 0; i < DSP_RING; ++i) {
                ctx.dsp.buf[i] = (uint8_t *)malloc(fsz);
                if (!ctx.dsp.buf[i]) { dio = 0; break; }
            }
            if (dio) {
                ctx.dsp.src_w = src_w; ctx.dsp.src_h = src_h;
                pthread_mutex_init(&ctx.dsp.mtx, NULL);
                pthread_cond_init(&ctx.dsp.cv_frame, NULL);
                pthread_cond_init(&ctx.dsp.cv_shot, NULL);
                pthread_cond_init(&ctx.dsp.cv_empty, NULL);
                ctx.dsp.started = 1;
                if (pthread_create(&ctx.dsp.tid, NULL, dsp_thread, &ctx.dsp) == 0) {
                    fprintf(stderr, "[adas] 显示流水已启用：独立线程，与推理并行"
                                    "（%dx%d 帧缓冲 ×%d）\n", src_w, src_h, DSP_RING);
                } else {
                    ctx.dsp.started = 0; dio = 0;
                }
            }
            if (!dio) {
                fprintf(stderr, "[adas] ⚠️  显示流水启动失败，关闭显示\n");
                for (int i = 0; i < DSP_RING; ++i) { free(ctx.dsp.buf[i]); ctx.dsp.buf[i] = NULL; }
                display_close(ctx.dsp.disp); ctx.dsp.disp = NULL;
            }
        }
    }

    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);
    arm_pipeline_start(pl);
    printf("▶ 已启动，Ctrl-C 退出\n\n");

    long last = 0;
    while (!g_stop) {
        usleep(500 * 1000);
        if (log_every > 0 && (long)(ctx.n_frames / (unsigned long)log_every) != last) {
            last = (long)(ctx.n_frames / (unsigned long)log_every);
            infer_stat_t st; infer_get_stat(&st);
            arm_pipeline_stats_t ps; arm_pipeline_stats(pl, &ps);
            printf("[%6.1fs] 帧=%lu 框/帧=%.1f | 预处理 %.1fms 推理 %.1fms 后处理 %.1fms = %.1fms (%.1f fps)"
                   " | 收帧 %llu 丢 %llu CRC错 %llu\n",
                   now_s() - ctx.t_first, ctx.n_frames,
                   ctx.n_frames ? (double)ctx.n_boxes_total / ctx.n_frames : 0.0,
                   st.avg_pre, st.avg_infer, st.avg_post, st.avg_total, st.avg_fps,
                   (unsigned long long)ps.frames_ok,
                   (unsigned long long)ps.frames_dropped,
                   (unsigned long long)ps.err_crc);
            fflush(stdout);
        }
        if (max_frames > 0 && (long)ctx.n_frames >= max_frames) break;
    }

    printf("\n⏹ 停止中...\n");
    arm_pipeline_stop(pl);
    if (ctx.pre.started) {
        pthread_mutex_lock(&ctx.pre.mtx);
        ctx.pre.stop = 1;
        pthread_cond_broadcast(&ctx.pre.cv_data);
        pthread_cond_broadcast(&ctx.pre.cv_space);
        pthread_mutex_unlock(&ctx.pre.mtx);
        pthread_join(ctx.pre.tid, NULL);
        ctx.pre.started = 0;
        for (int i = 0; i < PRE_RING; ++i) { free(ctx.pre.buf[i]); ctx.pre.buf[i] = NULL; }
        pthread_mutex_destroy(&ctx.pre.mtx);
        pthread_cond_destroy(&ctx.pre.cv_data);
        pthread_cond_destroy(&ctx.pre.cv_space);
    }
    if (ctx.dsp.disp && ctx.dsp.started) {
        pthread_mutex_lock(&ctx.dsp.mtx);
        ctx.dsp.stop = 1;
        pthread_cond_broadcast(&ctx.dsp.cv_frame);
        pthread_cond_broadcast(&ctx.dsp.cv_shot);
        pthread_cond_broadcast(&ctx.dsp.cv_empty);
        pthread_mutex_unlock(&ctx.dsp.mtx);
        pthread_join(ctx.dsp.tid, NULL);
        ctx.dsp.started = 0;
        if (ctx.dsp.n_renders > 0)
            fprintf(stderr, "[adas] 显示流水：画了 %d 帧，平均 %.2f ms，最慢 %.2f ms\n",
                    ctx.dsp.n_renders, ctx.dsp.t_sum / ctx.dsp.n_renders, ctx.dsp.t_max);
        for (int i = 0; i < DSP_RING; ++i) { free(ctx.dsp.buf[i]); ctx.dsp.buf[i] = NULL; }
        display_close(ctx.dsp.disp);
        ctx.dsp.disp = NULL;
        pthread_mutex_destroy(&ctx.dsp.mtx);
        pthread_cond_destroy(&ctx.dsp.cv_frame);
        pthread_cond_destroy(&ctx.dsp.cv_shot);
        pthread_cond_destroy(&ctx.dsp.cv_empty);
    }
    arm_pipeline_stats_print(pl);

    infer_stat_t st; infer_get_stat(&st);
    double el = now_s() - ctx.t_first;
    printf("\n============================================================\n");
    printf(" 汇总\n");
    printf("   运行时长     : %.1f s\n", el);
    printf("   处理帧数     : %lu\n", ctx.n_frames);
    printf("   平均帧率     : %.2f fps   %s\n", st.avg_fps,
           st.avg_fps >= 15.0 ? "✅ 达标（任务书 ≥15fps）" : "⚠️ 低于 15fps，需优化");
    if (ctx.quad == 2)
        printf("   单次推理(均) : 预处理 %.2f + 推理 %.2f + 后处理 %.2f = %.2f ms"
               "  ← 每帧跑 4 次（四路拼接）\n",
               st.avg_pre, st.avg_infer, st.avg_post, st.avg_total);
    else
        printf("   单帧耗时(均) : 预处理 %.2f + 推理 %.2f + 后处理 %.2f = %.2f ms\n",
               st.avg_pre, st.avg_infer, st.avg_post, st.avg_total);
    if (ctx.n_frames > 1 && ctx.t_last_frame > ctx.t_first_frame) {
        double span = ctx.t_last_frame - ctx.t_first_frame;
        double fps  = (double)(ctx.n_frames - 1) / span;
        printf("   稳态吞吐     : %.2f fps (%.2f ms/帧，首帧到末帧 %lu 个间隔 / %.2fs)\n",
               fps, 1000.0 / fps, ctx.n_frames - 1, span);
    }
    if (ctx.pre.cb_ms_total > 0 || ctx.pre.wk_ms_total > 0) {
        double span = (ctx.n_frames > 1 && ctx.t_last_frame > ctx.t_first_frame)
                      ? (ctx.t_last_frame - ctx.t_first_frame) * 1000.0 : 0.0;
        printf("   线程负载     : 预处理线程 %.0fms | 推理线程 %.0fms | 墙钟 %.0fms"
               "  -> %s\n",
               ctx.pre.cb_ms_total, ctx.pre.wk_ms_total, span,
               (span > 0 && ctx.pre.wk_ms_total > 0 &&
                span < 1.15 * (ctx.pre.wk_ms_total > ctx.pre.cb_ms_total
                               ? ctx.pre.wk_ms_total : ctx.pre.cb_ms_total))
                   ? "✅ 预处理已被推理完全覆盖" : "⚠️ 未完全重叠");
    }
    printf("   含收帧开销   : %.2f ms/帧 (%.2f fps 的墙钟倒数，差值为收帧+CRC+队列)\n",
           st.avg_frame_ms, st.avg_fps);
    printf("   端到端延迟   : %.1f ms   %s\n", st.avg_frame_ms,
           st.avg_frame_ms < 150.0 ? "✅ 达标（任务书 <150ms）" : "⚠️ 超标");
    printf("============================================================\n");

    arm_pipeline_free(pl);
    if (uart) {
        uart_stats_t us;
        uart_tx_stats(uart, &us);
        printf("   UART         : %llu 包 / %llu 字节 / 写错 %llu / 截断 %llu\n",
               (unsigned long long)us.packets, (unsigned long long)us.bytes,
               (unsigned long long)us.write_errors, (unsigned long long)us.truncated);
        uart_tx_close(uart);
    }
    infer_release();
    free(ctx.boxes);
    return 0;
}
