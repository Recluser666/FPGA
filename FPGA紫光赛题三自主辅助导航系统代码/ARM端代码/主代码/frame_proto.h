#ifndef FRAME_PROTO_H
#define FRAME_PROTO_H

#include <stdint.h>
#include <stddef.h>

#define FRAME_W            640
#define FRAME_H            360
#define FRAME_PIXELS       (FRAME_W * FRAME_H)
#define FRAME_HDR_LEN      14
#define FRAME_CRC_LEN      2
#define FRAME_TOTAL_LEN    (FRAME_HDR_LEN + FRAME_PIXELS + FRAME_CRC_LEN)

#define FRAME_MAGIC_0      0xA5
#define FRAME_MAGIC_1      0x5A

#pragma pack(push, 1)
typedef struct {
    uint8_t  magic0;
    uint8_t  magic1;
    uint16_t seq;
    uint16_t width;
    uint16_t height;
    uint32_t payload_len;
    uint16_t flags;
} frame_hdr_t;
#pragma pack(pop)

_Static_assert(sizeof(frame_hdr_t) == FRAME_HDR_LEN,
               "帧头必须是 14 字节 —— 改了结构体就要同步改代码");

static inline uint16_t be16(const uint8_t *p) {
    return (uint16_t)((p[0] << 8) | p[1]);
}
static inline uint32_t be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

#define FRAME_CRC_INIT     0xFFFF
#define FRAME_CRC_POLY     0x1021

static const uint16_t frame_crc16_tab[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

static inline uint16_t frame_crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = FRAME_CRC_INIT;
    for (size_t i = 0; i < len; ++i)
        crc = (uint16_t)((crc << 8) ^ frame_crc16_tab[((crc >> 8) ^ data[i]) & 0xFF]);
    return crc;
}

static inline uint16_t frame_crc16_ccitt_ref(const uint8_t *data, size_t len) {
    uint16_t crc = FRAME_CRC_INIT;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ FRAME_CRC_POLY)
                                 : (uint16_t)(crc << 1);
    }
    return crc;
}

typedef enum {
    FRAME_OK = 0,
    FRAME_ERR_MAGIC,
    FRAME_ERR_SIZE,
    FRAME_ERR_CRC,
} frame_status_t;

static inline const char *frame_status_str(frame_status_t s) {
    switch (s) {
        case FRAME_OK:       return "OK";
        case FRAME_ERR_MAGIC:return "MAGIC_MISMATCH";
        case FRAME_ERR_SIZE: return "SIZE_MISMATCH";
        case FRAME_ERR_CRC:  return "CRC_MISMATCH";
        default:             return "UNKNOWN";
    }
}

typedef enum {
    FRAME_PIX_GRAY8  = 0,
    FRAME_PIX_RGB565 = 1,
    FRAME_PIX_BGR565 = 2,
    FRAME_PIX_RGB888 = 3,
    FRAME_PIX_BGR888 = 4
} frame_pixfmt_t;

#define FRAME_FLAGS_PIXFMT_MASK   0x0007
#define FRAME_FLAGS_PIXFMT_SHIFT  0

static inline int frame_bytes_per_px(frame_pixfmt_t f) {
    switch (f) {
        case FRAME_PIX_GRAY8:  return 1;
        case FRAME_PIX_RGB565:
        case FRAME_PIX_BGR565: return 2;
        case FRAME_PIX_RGB888:
        case FRAME_PIX_BGR888: return 3;
        default:               return 0;
    }
}

#define FRAME_CTRL_LANE_SHIFT    3
#define FRAME_CTRL_MOTION_SHIFT  5
#define FRAME_CTRL_LLINE_SHIFT   7
#define FRAME_CTRL_RLINE_SHIFT   9
#define FRAME_CTRL_LIGHT_SHIFT  11
#define FRAME_CTRL_MASK(v)      ((v) & 0x03)

typedef struct {
    uint8_t lane;
    uint8_t motion;
    uint8_t lline;
    uint8_t rline;
    uint8_t light;
} frame_ctrl_t;

static inline frame_ctrl_t frame_ctrl_from_flags(uint16_t flags) {
    frame_ctrl_t c;
    c.lane   = (uint8_t)FRAME_CTRL_MASK(flags >> FRAME_CTRL_LANE_SHIFT);
    c.motion = (uint8_t)FRAME_CTRL_MASK(flags >> FRAME_CTRL_MOTION_SHIFT);
    c.lline  = (uint8_t)FRAME_CTRL_MASK(flags >> FRAME_CTRL_LLINE_SHIFT);
    c.rline  = (uint8_t)FRAME_CTRL_MASK(flags >> FRAME_CTRL_RLINE_SHIFT);
    c.light  = (uint8_t)FRAME_CTRL_MASK(flags >> FRAME_CTRL_LIGHT_SHIFT);
    return c;
}

static inline int frame_ctrl_equal(const frame_ctrl_t *a, const frame_ctrl_t *b) {
    return a->lane == b->lane && a->motion == b->motion &&
           a->lline == b->lline && a->rline == b->rline && a->light == b->light;
}

static inline uint16_t frame_flags_make(frame_pixfmt_t fmt, frame_ctrl_t c) {
    return (uint16_t)((fmt & 0x07)
        | ((c.lane   & 0x03) << FRAME_CTRL_LANE_SHIFT)
        | ((c.motion & 0x03) << FRAME_CTRL_MOTION_SHIFT)
        | ((c.lline  & 0x03) << FRAME_CTRL_LLINE_SHIFT)
        | ((c.rline  & 0x03) << FRAME_CTRL_RLINE_SHIFT)
        | ((c.light  & 0x03) << FRAME_CTRL_LIGHT_SHIFT));
}

static inline uint16_t frame_flags_merge(frame_pixfmt_t fmt, uint16_t ctrl_flags) {
    return (uint16_t)(((uint16_t)fmt & 0x07u) | (ctrl_flags & 0xFFF8u));
}

typedef struct {
    uint16_t       seq;
    uint16_t       width, height;
    frame_pixfmt_t fmt;
    uint32_t       payload_len;
    uint32_t       total_len;
    uint16_t       flags;
    frame_ctrl_t   ctrl;
} frame_info_t;

static inline frame_status_t frame_validate_ex(const uint8_t *buf, size_t frame_len,
                                               int require_crc, frame_info_t *out) {
    if (frame_len < FRAME_HDR_LEN) return FRAME_ERR_SIZE;
    if (buf[0] != FRAME_MAGIC_0 || buf[1] != FRAME_MAGIC_1)
        return FRAME_ERR_MAGIC;

    uint16_t w    = be16(buf + 4);
    uint16_t h    = be16(buf + 6);
    uint32_t plen = be32(buf + 8);
    uint16_t flags = be16(buf + 12);
    frame_pixfmt_t fmt = (frame_pixfmt_t)((flags >> FRAME_FLAGS_PIXFMT_SHIFT)
                                          & FRAME_FLAGS_PIXFMT_MASK);

    const int bpp = frame_bytes_per_px(fmt);
    if (w == 0 || h == 0 || bpp == 0) return FRAME_ERR_SIZE;
    if ((uint64_t)plen != (uint64_t)w * h * (uint64_t)bpp) return FRAME_ERR_SIZE;

    const size_t total = (size_t)FRAME_HDR_LEN + plen + FRAME_CRC_LEN;
    if (total > frame_len) return FRAME_ERR_SIZE;

    if (require_crc) {
        uint16_t calc = frame_crc16_ccitt(buf, FRAME_HDR_LEN + plen);
        uint16_t got  = be16(buf + FRAME_HDR_LEN + plen);
        if (calc != got) return FRAME_ERR_CRC;
    }

    if (out) {
        out->seq = be16(buf + 2);
        out->width = w; out->height = h;
        out->fmt = fmt;
        out->payload_len = plen;
        out->total_len = (uint32_t)total;
        out->flags = flags;
        out->ctrl  = frame_ctrl_from_flags(flags);
    }
    return FRAME_OK;
}

static inline size_t frame_total_len_of(int w, int h, frame_pixfmt_t fmt) {
    return (size_t)FRAME_HDR_LEN + (size_t)w * h * (size_t)frame_bytes_per_px(fmt)
           + FRAME_CRC_LEN;
}

static inline frame_status_t frame_validate(const uint8_t *buf,
                                            int require_crc,
                                            uint16_t *out_seq) {
    frame_info_t fi;
    frame_status_t s = frame_validate_ex(buf, FRAME_TOTAL_LEN, require_crc, &fi);
    if (s != FRAME_OK) return s;
    if (fi.width != FRAME_W || fi.height != FRAME_H ||
        fi.fmt != FRAME_PIX_GRAY8 || fi.payload_len != FRAME_PIXELS)
        return FRAME_ERR_SIZE;
    if (out_seq) *out_seq = fi.seq;
    return FRAME_OK;
}

static inline const uint8_t *frame_payload(const uint8_t *buf) {
    return buf + FRAME_HDR_LEN;
}

#endif
