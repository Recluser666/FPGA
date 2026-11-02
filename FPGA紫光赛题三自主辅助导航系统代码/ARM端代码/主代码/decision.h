#ifndef DECISION_H
#define DECISION_H

#include <stdint.h>

#include "postproc.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
    LIGHT_UNKNOWN = 0,
    LIGHT_RED     = 1,
    LIGHT_GREEN   = 2,
    LIGHT_YELLOW  = 3,
} light_state_t;

typedef enum {
    LANE_UNKNOWN  = 0,
    LANE_LEFT     = 1,
    LANE_STRAIGHT = 2,
    LANE_RIGHT    = 3,
} lane_state_t;

typedef enum {
    MOTION_UNKNOWN = 0,
    MOTION_RUNNING = 1,
    MOTION_STOPPED = 2,
} motion_state_t;

#define LED_NONE   0x00
#define LED_LEFT   0x01
#define LED_STRAIGHT 0x02
#define LED_RIGHT  0x04
#define LED_ALARM  0x08

typedef enum {
    LINE_UNKNOWN = 0,
    LINE_SOLID   = 1,
    LINE_DASHED  = 2,
} lane_line_t;

typedef enum {
    CMD_IDLE        = 0,
    CMD_GO_STRAIGHT = 1,
    CMD_TURN_LEFT   = 2,
    CMD_TURN_RIGHT  = 3,
    CMD_STOP        = 4,
    CMD_BRAKE       = 5,
    CMD_DECEL       = 6,
    CMD_PULL_OVER   = 7,
} cmd_t;

#define ALARM_NONE            0x00
#define ALARM_PED_REDLIGHT    0x01
#define ALARM_YELLOW_LIGHT    0x02
#define ALARM_OVERSPEED       0x04
#define ALARM_ILLEGAL_LANE    0x08
#define ALARM_RED_LIGHT       0x10
#define ALARM_COLLISION_RISK  0x20


typedef struct {
    light_state_t light;
    lane_state_t  lane;
    motion_state_t motion;
    lane_line_t   left_line;
    lane_line_t   right_line;
    uint16_t      speed_x10;
    uint16_t      speed_limit_x10;
    uint32_t      ts_ms;
} dec_input_ext_t;

typedef struct {
    int   img_w, img_h;

    float ped_zone_y_ratio;
    float ped_min_height_ratio;

    int   overspeed_margin_x10;

    uint32_t yellow_alarm_ms;

    uint32_t debounce_ms;
} dec_cfg_t;

typedef struct {
    cmd_t    cmd;
    uint8_t  alarms;
    uint8_t  arrow;
    uint8_t  leds;
    uint8_t  alarm_active;
    uint16_t flags;

    int      n_person;
    int      n_vehicle;
    int      ped_brake;
    int      overspeed;
} dec_output_t;

dec_cfg_t dec_default_cfg(int img_w, int img_h);


int dec_run(const pp_box_t *boxes, int n_boxes,
            const dec_input_ext_t *ext,
            const dec_cfg_t *cfg,
            dec_output_t *out);

const char *dec_cmd_name(cmd_t c);
const char *dec_alarm_str(uint8_t alarms, char *buf, int buflen);

#ifdef __cplusplus
}
#endif
#endif
