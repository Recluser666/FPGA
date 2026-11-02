#include "decision.h"

#include <stdio.h>
#include <string.h>

dec_cfg_t dec_default_cfg(int img_w, int img_h) {
    dec_cfg_t c;
    memset(&c, 0, sizeof(c));
    c.img_w = img_w > 0 ? img_w : 640;
    c.img_h = img_h > 0 ? img_h : 360;
    c.ped_zone_y_ratio     = 0.55f;
    c.ped_min_height_ratio = 0.05f;
    c.overspeed_margin_x10 = 50;
    c.yellow_alarm_ms      = 3000;
    c.debounce_ms          = 100;
    return c;
}

const char *dec_cmd_name(cmd_t c) {
    switch (c) {
        case CMD_IDLE:        return "IDLE";
        case CMD_GO_STRAIGHT: return "GO_STRAIGHT";
        case CMD_TURN_LEFT:   return "TURN_LEFT";
        case CMD_TURN_RIGHT:  return "TURN_RIGHT";
        case CMD_STOP:        return "STOP";
        case CMD_BRAKE:       return "BRAKE";
        case CMD_DECEL:       return "DECEL";
        case CMD_PULL_OVER:   return "PULL_OVER";
        default:              return "?";
    }
}

const char *dec_alarm_str(uint8_t alarms, char *buf, int buflen) {
    if (!buf || buflen <= 0) return "";
    buf[0] = '\0';
    if (alarms == ALARM_NONE) { snprintf(buf, buflen, "无"); return buf; }
    struct { uint8_t bit; const char *n; } t[] = {
        {ALARM_PED_REDLIGHT,   "行人闯红灯"},
        {ALARM_RED_LIGHT,      "红灯"},
        {ALARM_YELLOW_LIGHT,   "黄灯"},
        {ALARM_OVERSPEED,      "超速"},
        {ALARM_ILLEGAL_LANE,   "实线变道"},
        {ALARM_COLLISION_RISK, "碰撞风险"},
    };
    int first = 1;
    for (size_t i = 0; i < sizeof(t) / sizeof(t[0]); ++i) {
        if (alarms & t[i].bit) {
            if (!first) strncat(buf, "+", buflen - strlen(buf) - 1);
            strncat(buf, t[i].n, buflen - strlen(buf) - 1);
            first = 0;
        }
    }
    return buf;
}

static int ped_is_near(const pp_box_t *b, const dec_cfg_t *cfg) {
    if (b->cls != PP_CLS_PERSON) return 0;
    int h = b->y2 - b->y1;
    if (h <= 0) return 0;
    float h_ratio  = (float)h / (float)cfg->img_h;
    float ybot_ratio = (float)b->y2 / (float)cfg->img_h;
    return (h_ratio >= cfg->ped_min_height_ratio) &&
           (ybot_ratio >= cfg->ped_zone_y_ratio);
}

int dec_run(const pp_box_t *boxes, int n_boxes,
            const dec_input_ext_t *ext,
            const dec_cfg_t *cfg,
            dec_output_t *out) {
    if (!cfg || !out) return -1;

    memset(out, 0, sizeof(*out));

    int ped_near = 0;
    for (int i = 0; i < n_boxes; ++i) {
        switch (boxes[i].cls) {
            case PP_CLS_PERSON: ped_near += ped_is_near(&boxes[i], cfg);
                                out->n_person++; break;
            case PP_CLS_CAR:    out->n_vehicle++; break;
            default: break;
        }
    }

    light_state_t  light  = LIGHT_UNKNOWN;
    lane_state_t   lane   = LANE_UNKNOWN;
    motion_state_t motion = MOTION_UNKNOWN;
    lane_line_t    lline  = LINE_UNKNOWN, rline = LINE_UNKNOWN;
    uint16_t speed = 0, limit = 0;
    if (ext) {
        light = ext->light; lane = ext->lane; motion = ext->motion;
        lline = ext->left_line; rline = ext->right_line;
        speed = ext->speed_x10; limit = ext->speed_limit_x10;
    }

    if (light == LIGHT_UNKNOWN) {
        int red = 0, grn = 0, yel = 0;
        for (int i = 0; i < n_boxes; ++i) {
            if (boxes[i].cls == PP_CLS_TL_RED)    red++;
            if (boxes[i].cls == PP_CLS_TL_GREEN)  grn++;
            if (boxes[i].cls == PP_CLS_TL_YELLOW) yel++;
        }
        if (red > 0)      light = LIGHT_RED;
        else if (yel > 0) light = LIGHT_YELLOW;
        else if (grn > 0) light = LIGHT_GREEN;
    }

    uint8_t leds = LED_NONE;
    uint8_t arrow = 0;
    switch (lane) {
        case LANE_LEFT:     leds = LED_LEFT;     arrow = 1; break;
        case LANE_RIGHT:    leds = LED_RIGHT;    arrow = 3; break;
        case LANE_STRAIGHT: leds = LED_STRAIGHT; arrow = 2; break;
        default:            leds = LED_STRAIGHT; arrow = 2; break;
    }


    if (light == LIGHT_RED && ped_near > 0) {
        out->cmd = CMD_BRAKE;
        out->alarms |= ALARM_PED_REDLIGHT;
        out->ped_brake = 1;
        out->alarm_active = 1;
        out->leds = LED_ALARM;
        out->arrow = 0;
        return 0;
    }

    if (light == LIGHT_RED) {
        out->cmd = CMD_STOP;
        out->alarms |= ALARM_RED_LIGHT;
        out->leds = LED_NONE;
        out->arrow = 0;
        return 0;
    }

    if (limit > 0 && speed > 0 &&
        (int)speed - (int)limit > cfg->overspeed_margin_x10) {
        out->cmd = CMD_DECEL;
        out->alarms |= ALARM_OVERSPEED;
        out->overspeed = 1;
    }

    if (lane == LANE_LEFT && lline == LINE_SOLID) {
        out->cmd = CMD_PULL_OVER;
        out->alarms |= ALARM_ILLEGAL_LANE;
        out->alarm_active = 1;
        out->leds = LED_ALARM;
        out->arrow = 0;
        return 0;
    }
    if (lane == LANE_RIGHT && rline == LINE_SOLID) {
        out->cmd = CMD_PULL_OVER;
        out->alarms |= ALARM_ILLEGAL_LANE;
        out->alarm_active = 1;
        out->leds = LED_ALARM;
        out->arrow = 0;
        return 0;
    }

    if (light == LIGHT_YELLOW) {
        out->alarms |= ALARM_YELLOW_LIGHT;
        out->alarm_active = 1;
    }

    if (out->cmd == CMD_IDLE || out->cmd == CMD_DECEL) {
        cmd_t base;
        switch (lane) {
            case LANE_LEFT:  base = CMD_TURN_LEFT;  break;
            case LANE_RIGHT: base = CMD_TURN_RIGHT; break;
            default:         base = CMD_GO_STRAIGHT; break;
        }
        if (out->cmd != CMD_DECEL) out->cmd = base;
    }
    out->leds = leds;
    out->arrow = arrow;

    if (out->alarm_active) out->leds |= LED_ALARM;

    if (motion == MOTION_STOPPED) {
        out->cmd = CMD_STOP;
        out->arrow = 0;
        out->leds = LED_NONE;
        if (out->alarm_active) out->leds = LED_ALARM;
        return 0;
    }

    return 0;
}
