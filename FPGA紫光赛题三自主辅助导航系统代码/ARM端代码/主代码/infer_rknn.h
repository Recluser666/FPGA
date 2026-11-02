#ifndef INFER_RKNN_H
#define INFER_RKNN_H

#include <stdint.h>
#include "infer_pre.h"
#include "postproc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define INFER_MAX_HEADS 3

typedef struct {
    const char   *model_path;
    int           img_size;
    int           nc;
    float         conf_thres;
    float         iou_thres;
    int           max_det;
    int           min_box_px;
    int           nms_agnostic;
    int           out_has_sigmoid;
    const float  *anchors;
    int           n_heads;
} infer_cfg_t;

infer_cfg_t infer_default_cfg(const char *model_path);

typedef struct {
    unsigned long frames;
    double ms_pre;
    double ms_infer;
    double ms_post;
    double ms_total;

    double avg_pre, avg_infer, avg_post, avg_total;
    double avg_fps;
    double avg_frame_ms;
    int    last_n_boxes;
} infer_stat_t;

int  infer_init(const infer_cfg_t *cfg);

int  infer_run(const uint8_t *pix, int src_w, int src_h, infer_pixfmt_t fmt,
               pp_box_t *out, int max_out, int *n_out);

int  infer_run_ex(const uint8_t *pix, int src_w, int src_h, infer_pixfmt_t fmt,
                  pp_box_t *out, int max_out, int *n_out, int is_frame);


int  infer_input_dims(int *w, int *h);

int  infer_pre_stage(const uint8_t *pix, int src_w, int src_h, infer_pixfmt_t fmt,
                     uint8_t *dst, infer_lb_t *lb_out);

int  infer_run_stage(const infer_lb_t *lb, const uint8_t *inbuf,
                     pp_box_t *out, int max_out, int *n_out,
                     int is_frame, double t_pre_ms);

void infer_release(void);
void infer_get_stat(infer_stat_t *out);
int  infer_is_ready(void);

#ifdef __cplusplus
}
#endif
#endif
