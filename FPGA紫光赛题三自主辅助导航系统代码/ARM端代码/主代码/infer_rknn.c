#include "infer_rknn.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef HAVE_RKNN
#include "rknn_api.h"
#endif

static infer_cfg_t  g_cfg;
static int          g_ready = 0;
static int g_in_w = 640, g_in_h = 640;

static infer_stat_t g_stat;
static double       g_t_start = 0.0;
static double       g_sum_pre = 0.0, g_sum_infer = 0.0;
static double       g_sum_post = 0.0, g_sum_total = 0.0;

static uint8_t     *g_inbuf = NULL;
static double       g_t_first = 0;

static int g_n_heads = 0;
static int g_hw[INFER_MAX_HEADS];
static int   g_ozp[INFER_MAX_HEADS];
static float g_oscale[INFER_MAX_HEADS];
static int   g_want_float = 1;
static int g_ww[INFER_MAX_HEADS];

static int g_stride[INFER_MAX_HEADS];

static pp_box_t    *g_cand = NULL;

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

infer_cfg_t infer_default_cfg(const char *model_path) {
    infer_cfg_t c;
    memset(&c, 0, sizeof(c));
    c.model_path      = model_path;
    c.img_size        = 640;
    c.nc              = 5;
    c.conf_thres      = 0.25f;
    c.iou_thres       = 0.45f;
    c.max_det         = 100;
    c.min_box_px      = 3;
    c.nms_agnostic    = 0;
    c.out_has_sigmoid = 1;
    c.anchors         = NULL;
    c.n_heads         = 3;
    return c;
}

int infer_is_ready(void) { return g_ready; }

void infer_get_stat(infer_stat_t *out) {
    if (out) *out = g_stat;
}

#ifdef HAVE_RKNN
static rknn_context g_ctx;
static rknn_output  g_outs[INFER_MAX_HEADS];
static int          g_n_out = 0;
#endif

int infer_init(const infer_cfg_t *cfg) {
    if (!cfg || !cfg->model_path || cfg->img_size <= 0 || cfg->nc <= 0) return -1;
    if (g_ready) infer_release();

    g_cfg = *cfg;
    if (!g_cfg.anchors) g_cfg.anchors = pp_yolov5n_default_anchors();
    if (g_cfg.n_heads <= 0) g_cfg.n_heads = 3;
    g_n_heads = g_cfg.n_heads;

    g_cand = (pp_box_t *)malloc(sizeof(pp_box_t) * (size_t)g_cfg.max_det * 4);
    if (!g_cand) { infer_release(); return -3; }

#ifndef HAVE_RKNN
    g_in_w = g_in_h = g_cfg.img_size;
    g_inbuf = (uint8_t *)malloc((size_t)g_in_w * g_in_h * 3);
    if (!g_inbuf) { infer_release(); return -3; }
    fprintf(stderr, "[infer] 未编译 RKNN 支持（缺 -DHAVE_RKNN），仅纯 C 路径可用\n");
    return -2;
#else
    int ret = rknn_init(&g_ctx, (void *)g_cfg.model_path, 0, 0, NULL);
    if (ret < 0) { fprintf(stderr, "[infer] rknn_init 失败 %d\n", ret); infer_release(); return -3; }

    rknn_input_output_num io_num;
    ret = rknn_query(g_ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret < 0) { fprintf(stderr, "[infer] 查询 in/out 数失败 %d\n", ret); infer_release(); return -3; }
    g_n_out = (int)io_num.n_output;
    if (g_n_out > INFER_MAX_HEADS) g_n_out = INFER_MAX_HEADS;

    rknn_tensor_attr in_attr;
    memset(&in_attr, 0, sizeof(in_attr));
    in_attr.index = 0;
    ret = rknn_query(g_ctx, RKNN_QUERY_INPUT_ATTR, &in_attr, sizeof(in_attr));
    if (ret < 0) { fprintf(stderr, "[infer] 查询输入属性失败 %d\n", ret); infer_release(); return -3; }

    fprintf(stderr, "[infer] 输入: n_dims=%d [%d,%d,%d,%d] fmt=%d type=%d\n",
            in_attr.n_dims, in_attr.dims[0], in_attr.dims[1], in_attr.dims[2],
            in_attr.dims[3], in_attr.fmt, in_attr.type);

    if (in_attr.n_dims >= 4) {
        g_in_h = in_attr.dims[1];
        g_in_w = in_attr.dims[2];
    } else {
        g_in_h = g_in_w = g_cfg.img_size;
    }
    if (g_in_w <= 0 || g_in_h <= 0) {
        fprintf(stderr, "[infer] ✖ 模型输入尺寸异常 %dx%d\n", g_in_w, g_in_h);
        infer_release(); return -3;
    }
    if (g_in_w != g_cfg.img_size || g_in_h != g_cfg.img_size)
        fprintf(stderr, "[infer] 注: 模型输入 %dx%d 与配置的 %dx%d 不同，按模型为准\n",
                g_in_w, g_in_h, g_cfg.img_size, g_cfg.img_size);

    g_inbuf = (uint8_t *)malloc((size_t)g_in_w * (size_t)g_in_h * 3);
    if (!g_inbuf) {
        fprintf(stderr, "[infer] ✖ 输入缓冲分配失败（%d x %d x 3）\n", g_in_w, g_in_h);
        infer_release(); return -3;
    }

    for (int i = 0; i < g_n_out; ++i) {
        rknn_tensor_attr oa;
        memset(&oa, 0, sizeof(oa));
        oa.index = i;
        if (rknn_query(g_ctx, RKNN_QUERY_OUTPUT_ATTR, &oa, sizeof(oa)) < 0) {
            fprintf(stderr, "[infer] 查询输出 %d 属性失败\n", i);
            infer_release(); return -3;
        }
        int C = oa.dims[1], H = oa.dims[2], W = oa.dims[3];
        g_hw[i] = H;
        g_ww[i] = W;
        g_oscale[i] = oa.scale;
        g_ozp[i]   = (int)oa.zp;
        fprintf(stderr, "[infer] 输出%d 量化: scale=%.6f zp=%d\n", i, oa.scale, (int)oa.zp);
        g_stride[i] = g_in_h / H;
        fprintf(stderr, "[infer] 输出%d: C=%d H=%d W=%d -> stride=%d\n", i, C, H, W, g_stride[i]);
        const int per_anchor = g_cfg.nc + 5;
        if (C <= 0 || C % per_anchor != 0) {
            fprintf(stderr, "[infer] ✖ 输出通道 %d 不是 (nc+5)=%d 的整数倍，模型与配置不匹配\n",
                    C, per_anchor);
        } else if (i == 0) {
            fprintf(stderr, "[infer] ✔ 每格点 %d 通道 = %d 锚框 × %d (5+%d类)\n",
                    C, C / per_anchor, per_anchor, g_cfg.nc);
        }
    }
    {
        const char *e = getenv("ADAS_WANT_FLOAT");
        g_want_float = (e && *e == '1') ? 1 : 0;
        if (!g_want_float)
            fprintf(stderr, "[infer] want_float=0：直接读 int8 输出，解码时反量化（省一趟转换）\n");
    }
    g_ready = 1;
    g_stat = (infer_stat_t){0};
    g_sum_pre = g_sum_infer = g_sum_post = g_sum_total = 0.0;
    g_t_start = now_ms();
    g_t_first = 0;
    fprintf(stderr, "[infer] ✅ 就绪: %s  %d×%d  %d 类\n",
            g_cfg.model_path, g_in_w, g_in_h, g_cfg.nc);
    return 0;
#endif
}


int infer_input_dims(int *w, int *h) {
    if (!g_ready) return -2;
    if (w) *w = g_in_w;
    if (h) *h = g_in_h;
    return 0;
}

int infer_pre_stage(const uint8_t *pix, int src_w, int src_h, infer_pixfmt_t fmt,
                    uint8_t *dst, infer_lb_t *lb_out) {
    if (!pix || !dst || src_w <= 0 || src_h <= 0) return -1;
    if (!g_ready) return -2;
    return infer_preprocess_ex(pix, src_w, src_h, fmt, dst,
                               g_in_w, g_in_h, lb_out) == 0 ? 0 : -1;
}

int infer_run_stage(const infer_lb_t *lb, const uint8_t *inbuf,
                    pp_box_t *out, int max_out, int *n_out,
                    int is_frame, double t_pre_ms) {
    if (n_out) *n_out = 0;
    if (!lb || !inbuf || !out || max_out <= 0) return -1;
    if (!g_ready) return -2;

    const double t0 = now_ms();
#ifdef HAVE_RKNN
    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index        = 0;
    inputs[0].type         = RKNN_TENSOR_UINT8;
    inputs[0].size         = (uint32_t)((size_t)g_in_w * (size_t)g_in_h * 3);
    inputs[0].fmt          = RKNN_TENSOR_NHWC;
    inputs[0].buf          = (void *)inbuf;
    inputs[0].pass_through = 0;

    int ret = rknn_inputs_set(g_ctx, 1, inputs);
    if (ret < 0) { fprintf(stderr, "[infer] inputs_set %d\n", ret); return -3; }

    ret = rknn_run(g_ctx, NULL);
    if (ret < 0) { fprintf(stderr, "[infer] run %d\n", ret); return -3; }

    memset(g_outs, 0, sizeof(g_outs));
    for (int i = 0; i < g_n_out; ++i) {
        g_outs[i].index      = i;
        g_outs[i].want_float = (uint8_t)g_want_float;
    }
    ret = rknn_outputs_get(g_ctx, g_n_out, g_outs, NULL);
    if (ret < 0) { fprintf(stderr, "[infer] outputs_get %d\n", ret); return -3; }
    const double t2 = now_ms();

    pp_head_t heads[INFER_MAX_HEADS];
    memset(heads, 0, sizeof(heads));
    for (int i = 0; i < g_n_out; ++i) {
        heads[i].data   = (const float *)g_outs[i].buf;
        if (!g_want_float) {
            heads[i].qdata = (const int8_t *)g_outs[i].buf;
            heads[i].scale = g_oscale[i];
            heads[i].zp    = g_ozp[i];
        }
        heads[i].C      = (int)(g_outs[i].size /
                          (g_want_float ? sizeof(float) : sizeof(int8_t))) /
                          (g_hw[i] * g_ww[i]);
        heads[i].H      = g_hw[i];
        heads[i].W      = g_ww[i];
        heads[i].stride = g_stride[i];
    }

    pp_lb_t pplb;
    pplb.src_w = lb->src_w; pplb.src_h = lb->src_h;
    pplb.dst_w = lb->dst_w; pplb.dst_h = lb->dst_h;
    pplb.ratio = lb->ratio; pplb.pad_x = lb->pad_x; pplb.pad_y = lb->pad_y;

    int ncand = pp_decode_std(heads, g_n_out, g_cfg.nc, g_cfg.conf_thres,
                              g_cfg.anchors, g_cfg.out_has_sigmoid,
                              &pplb, g_cand, g_cfg.max_det * 4);
    if (ncand < 0) ncand = 0;

    pp_cfg_t pcfg = pp_default_cfg();
    pcfg.conf_thres = g_cfg.conf_thres;
    pcfg.iou_thres  = g_cfg.iou_thres;
    pcfg.max_det    = g_cfg.max_det;
    pcfg.min_box_px = g_cfg.min_box_px;
    pcfg.agnostic   = g_cfg.nms_agnostic;

    int n = pp_finish(g_cand, ncand, &pcfg, out, max_out);
    if (n < 0) n = 0;

    if (n > 0) {
        pp_light_opt_t lo = pp_light_default_opt();
        { const char *e = getenv("ADAS_TL_MINW"); if (e && *e) lo.min_w_in = atoi(e); }
        { const char *e = getenv("ADAS_TL_NOFIX"); if (e && *e == '1') lo.do_color = 0; }
        n = pp_refine_lights(inbuf, g_in_w, g_in_h, &pplb, out, n, &lo);
    }

    rknn_outputs_release(g_ctx, g_n_out, g_outs);
    const double t3 = now_ms();

    if (is_frame)
        g_stat.frames++;
    g_stat.ms_pre   = t_pre_ms;
    g_stat.ms_infer = t2 - t0;
    g_stat.ms_post  = t3 - t2;
    g_stat.ms_total = t_pre_ms + (t3 - t0);
    g_stat.last_n_boxes = n;

    g_sum_pre   += g_stat.ms_pre;
    g_sum_infer += g_stat.ms_infer;
    g_sum_post  += g_stat.ms_post;
    g_sum_total += g_stat.ms_total;
    if (g_stat.frames > 0) {
        g_stat.avg_pre   = g_sum_pre   / (double)g_stat.frames;
        g_stat.avg_infer = g_sum_infer / (double)g_stat.frames;
        g_stat.avg_post  = g_sum_post  / (double)g_stat.frames;
        g_stat.avg_total = g_sum_total / (double)g_stat.frames;
    }
    if (g_stat.frames == 1) g_t_first = now_ms();
    double elapsed = g_t_first > 0 ? now_ms() - g_t_first : 0.0;
    double n_iv = (g_stat.frames > 1) ? (double)(g_stat.frames - 1) : 1.0;
    g_stat.avg_fps = elapsed > 0 ? n_iv * 1000.0 / elapsed : 0.0;
    g_stat.avg_frame_ms = g_stat.avg_fps > 0 ? 1000.0 / g_stat.avg_fps : 0.0;

    if (n_out) *n_out = n;
    return 0;
#else
    (void)out; (void)max_out; (void)t_pre_ms; (void)is_frame; (void)lb; (void)inbuf;
    (void)t0;
    (void)g_hw; (void)g_stride; (void)g_t_start; (void)g_n_heads;
    return -2;
#endif
}

int infer_run_ex(const uint8_t *pix, int src_w, int src_h, infer_pixfmt_t fmt,
                 pp_box_t *out, int max_out, int *n_out, int is_frame) {
    if (n_out) *n_out = 0;
    if (!pix || !out || max_out <= 0 || src_w <= 0 || src_h <= 0) return -1;
    if (!g_ready) return -2;
    infer_lb_t lb;
    const double t0 = now_ms();
    if (!g_inbuf) return -2;
    if (infer_pre_stage(pix, src_w, src_h, fmt, g_inbuf, &lb) != 0) return -1;
    const double t_pre = now_ms() - t0;
    return infer_run_stage(&lb, g_inbuf, out, max_out, n_out, is_frame, t_pre);
}

void infer_release(void) {
#ifdef HAVE_RKNN
    if (g_ready) {
        rknn_destroy(g_ctx);
    }
#endif
    g_ready = 0;
    free(g_inbuf); g_inbuf = NULL;
    free(g_cand);  g_cand  = NULL;
    g_n_heads = 0;
    memset(&g_stat, 0, sizeof(g_stat));
    g_t_first = 0;
}

int infer_run(const uint8_t *pix, int src_w, int src_h, infer_pixfmt_t fmt,
              pp_box_t *out, int max_out, int *n_out)
{
    return infer_run_ex(pix, src_w, src_h, fmt, out, max_out, n_out, 1);
}
