#ifndef INFER_PRE_H
#define INFER_PRE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    INFER_PIX_GRAY8   = 0,
    INFER_PIX_RGB565  = 1,
    INFER_PIX_BGR565  = 2,
    INFER_PIX_RGB888  = 3,
    INFER_PIX_BGR888  = 4
} infer_pixfmt_t;

int infer_bytes_per_px(infer_pixfmt_t fmt);

typedef struct {
    int   src_w, src_h;
    int   dst_w, dst_h;
    float ratio;
    int   pad_x, pad_y;
} infer_lb_t;

infer_lb_t infer_letterbox_param(int src_w, int src_h, int dst_w, int dst_h);

#define INFER_PAD_VALUE 114

int infer_preprocess(const uint8_t *src, int src_w, int src_h, infer_pixfmt_t fmt,
                     uint8_t *dst, int img_size, infer_lb_t *lb_out);

int infer_preprocess_ex(const uint8_t *src, int src_w, int src_h, infer_pixfmt_t fmt,
                        uint8_t *dst, int dst_w, int dst_h, infer_lb_t *lb_out);

void infer_pixel_to_rgb(const uint8_t *p, infer_pixfmt_t fmt, uint8_t *r, uint8_t *g, uint8_t *b);

#ifdef __cplusplus
}
#endif
#endif
