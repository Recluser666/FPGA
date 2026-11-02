#ifndef POSTPROC_H
#define POSTPROC_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PP_CLS_PERSON     0
#define PP_CLS_CAR        1
#define PP_CLS_TL_RED     2
#define PP_CLS_TL_GREEN   3
#define PP_CLS_TL_YELLOW  4
#define PP_NUM_CLASSES    5

#define PP_CLS_BUS        PP_CLS_CAR
#define PP_CLS_TRUCK      PP_CLS_CAR

typedef struct {
    int      cls;
    float    conf;
    int      x1, y1;
    int      x2, y2;
} pp_box_t;

typedef struct {
    int    src_w, src_h;
    int    dst_w, dst_h;
    float  ratio;
    int    pad_x, pad_y;
} pp_lb_t;

typedef struct {
    float conf_thres;
    float iou_thres;
    int   max_det;
    int   agnostic;
    int   min_box_px;
} pp_cfg_t;

pp_cfg_t pp_default_cfg(void);


pp_lb_t pp_letterbox_param(int src_w, int src_h, int dst_w, int dst_h);

int pp_letterbox(const unsigned char *src, int src_w, int src_h,
                 unsigned char *dst, const pp_lb_t *lb, int pad_value);


int pp_map_back(float cx_n, float cy_n, float w_n, float h_n,
                const pp_lb_t *lb, float *x1, float *y1, float *x2, float *y2);







typedef struct {
    const float *data;
    int   C;
    int   H, W;
    int   stride;

    const int8_t *qdata;
    float scale;
    int   zp;
} pp_head_t;

static inline float pp_head_val(const pp_head_t *H, size_t idx) {
    if (H->qdata) return ((float)H->qdata[idx] - (float)H->zp) * H->scale;
    return H->data[idx];
}

typedef struct {
    float contain;
    int   min_w_in;
    int   do_color;
} pp_light_opt_t;

pp_light_opt_t pp_light_default_opt(void);

int pp_refine_lights(const uint8_t *rgb, int w, int h, const pp_lb_t *lb,
                     pp_box_t *boxes, int n, const pp_light_opt_t *opt);

int pp_decode_std(const pp_head_t *heads, int n_heads,
                  int nc, float conf_thres,
                  const float *anchors,
                  int out_has_sigmoid,
                  const pp_lb_t *lb,
                  pp_box_t *out, int max_out);

const float *pp_yolov5n_default_anchors(void);


int pp_nms(pp_box_t *boxes, int n, float iou_thres, int agnostic,
           pp_box_t *out, int max_out);


int pp_finish(pp_box_t *cand, int n, const pp_cfg_t *cfg,
              pp_box_t *out, int max_out);

float pp_iou(const pp_box_t *a, const pp_box_t *b);

#ifdef __cplusplus
}
#endif
#endif
