#ifndef FRAME_PARSE_H
#define FRAME_PARSE_H

#include <pthread.h>
#include <stdint.h>
#include <stdatomic.h>

#include "frame_proto.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct arm_transport {
    const char *name;
    int  (*open)(struct arm_transport *self);
    int  (*read)(struct arm_transport *self, uint8_t *buf, size_t len);
    void (*close)(struct arm_transport *self);
    void *priv;
} arm_transport_t;


arm_transport_t *arm_transport_spi_new(const char *dev, uint32_t speed_hz);

arm_transport_t *arm_transport_file_new_ex(const char *path, uint64_t max_frames);
arm_transport_t *arm_transport_file_new(const char *path);

arm_transport_t *arm_transport_raw_new(const char *path, int w, int h,
                                       frame_pixfmt_t fmt);

arm_transport_t *arm_transport_dma_new(const char *dev_path, const char *bar_path,
                                       int frame_w, int frame_h,
                                       frame_pixfmt_t fmt);

arm_transport_t *arm_transport_pcie_new(const char *resource_path, size_t window,
                                        volatile uint32_t *doorbell_reg);


typedef void (*arm_infer_cb)(const uint8_t *gray, int w, int h,
                            uint16_t seq, void *ctx);

typedef void (*arm_ctrl_cb)(const frame_ctrl_t *ctrl, void *ctx);


typedef struct {
    uint64_t frames_ok;
    uint64_t err_magic;
    uint64_t err_size;
    uint64_t err_crc;
    uint64_t frames_lost;
    uint64_t frames_ooo;
    uint64_t frames_dropped;
    uint64_t bytes_read;
    double   recv_ms_avg;
    double   recv_ms_max;
    double   infer_ms_avg;
    double   infer_ms_max;
    uint16_t last_seq;
} arm_pipeline_stats_t;


typedef struct arm_pipeline arm_pipeline_t;

void arm_pipeline_set_ctrl_cb(arm_pipeline_t *pl, arm_ctrl_cb cb, void *ctx);

arm_pipeline_t *arm_pipeline_new(arm_transport_t *transport,
                                 arm_infer_cb infer_cb, void *infer_ctx,
                                 int require_crc);

arm_pipeline_t *arm_pipeline_new_ex(arm_transport_t *transport,
                                    arm_infer_cb infer_cb, void *infer_ctx,
                                    int require_crc,
                                    int frame_w, int frame_h,
                                    frame_pixfmt_t frame_fmt);

int  arm_pipeline_start(arm_pipeline_t *pl);
void arm_pipeline_stop(arm_pipeline_t *pl);
void arm_pipeline_free(arm_pipeline_t *pl);

void arm_pipeline_stats(const arm_pipeline_t *pl, arm_pipeline_stats_t *out);

void arm_pipeline_stats_print(const arm_pipeline_t *pl);

#ifdef __cplusplus
}
#endif
#endif
