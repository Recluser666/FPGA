#include "postproc.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

pp_cfg_t pp_default_cfg(void) {
    pp_cfg_t c;
    c.conf_thres = 0.25f;
    c.iou_thres  = 0.45f;
    c.max_det    = 100;
    c.agnostic   = 0;
    c.min_box_px = 3;
    return c;
}

static inline float sigmoid_f(float x) {
    if (x >= 0.0f) return 1.0f / (1.0f + expf(-x));
    float e = expf(x);
    return e / (1.0f + e);
}


pp_lb_t pp_letterbox_param(int src_w, int src_h, int dst_w, int dst_h) {
    pp_lb_t lb;
    lb.src_w = src_w; lb.src_h = src_h;
    lb.dst_w = dst_w; lb.dst_h = dst_h;
    float r = (float)dst_w / (float)src_w;
    float r2 = (float)dst_h / (float)src_h;
    if (r2 < r) r = r2;
    lb.ratio = r;
    int nw = (int)lroundf((float)src_w * r);
    int nh = (int)lroundf((float)src_h * r);
    lb.pad_x = (dst_w - nw) / 2;
    lb.pad_y = (dst_h - nh) / 2;
    return lb;
}

int pp_letterbox(const unsigned char *src, int src_w, int src_h,
                 unsigned char *dst, const pp_lb_t *lb, int pad_value) {
    if (!src || !dst || !lb) return -1;
    if (src_w != lb->src_w || src_h != lb->src_h) return -1;

    memset(dst, pad_value, (size_t)lb->dst_w * lb->dst_h);

    int nw = (int)lroundf((float)src_w * lb->ratio);
    int nh = (int)lroundf((float)src_h * lb->ratio);

    for (int y = 0; y < nh; ++y) {
        int sy = (int)((float)y / lb->ratio);
        if (sy >= src_h) sy = src_h - 1;
        const unsigned char *srow = src + (size_t)sy * src_w;
        unsigned char *drow = dst + (size_t)(y + lb->pad_y) * lb->dst_w + lb->pad_x;
        for (int x = 0; x < nw; ++x) {
            int sx = (int)((float)x / lb->ratio);
            if (sx >= src_w) sx = src_w - 1;
            drow[x] = srow[sx];
        }
    }
    return 0;
}


static int map_back_px(float cx, float cy, float bw, float bh,
                       const pp_lb_t *lb, float *x1, float *y1,
                       float *x2, float *y2) {
    if (!lb || lb->ratio <= 0.0f) return -1;

    float bx1 = cx - bw * 0.5f, by1 = cy - bh * 0.5f;
    float bx2 = cx + bw * 0.5f, by2 = cy + bh * 0.5f;

    bx1 -= lb->pad_x;  bx2 -= lb->pad_x;
    by1 -= lb->pad_y;  by2 -= lb->pad_y;

    bx1 /= lb->ratio;  bx2 /= lb->ratio;
    by1 /= lb->ratio;  by2 /= lb->ratio;

    if (bx1 < 0) bx1 = 0;
    if (by1 < 0) by1 = 0;
    if (bx2 > lb->src_w) bx2 = lb->src_w;
    if (by2 > lb->src_h) by2 = lb->src_h;

    *x1 = bx1; *y1 = by1; *x2 = bx2; *y2 = by2;
    return 0;
}

int pp_map_back(float cx_n, float cy_n, float w_n, float h_n,
                const pp_lb_t *lb, float *x1, float *y1, float *x2, float *y2) {
    if (!lb) return -1;
    return map_back_px(cx_n * lb->dst_w, cy_n * lb->dst_h,
                       w_n * lb->dst_w, h_n * lb->dst_h,
                       lb, x1, y1, x2, y2);
}


float pp_iou(const pp_box_t *a, const pp_box_t *b) {
    int ix1 = a->x1 > b->x1 ? a->x1 : b->x1;
    int iy1 = a->y1 > b->y1 ? a->y1 : b->y1;
    int ix2 = a->x2 < b->x2 ? a->x2 : b->x2;
    int iy2 = a->y2 < b->y2 ? a->y2 : b->y2;
    int iw = ix2 - ix1, ih = iy2 - iy1;
    if (iw <= 0 || ih <= 0) return 0.0f;
    float inter = (float)iw * (float)ih;
    float ua = (float)(a->x2 - a->x1) * (float)(a->y2 - a->y1);
    float ub = (float)(b->x2 - b->x1) * (float)(b->y2 - b->y1);
    float uni = ua + ub - inter;
    return uni > 0.0f ? inter / uni : 0.0f;
}


int pp_decode_std(const pp_head_t *heads, int n_heads,
                           int nc, float conf_thres,
                           const float *anchors,
                           int out_has_sigmoid,
                           const pp_lb_t *lb,
                           pp_box_t *out, int max_out) {
    if (!heads || !anchors || !out || max_out <= 0 || nc <= 0) return -1;
    const int nAttr = nc + 5;
    int written = 0;

    for (int h = 0; h < n_heads; ++h) {
        const pp_head_t *H = &heads[h];
        const int nA = H->C / nAttr;
        if (nA <= 0 || nA * nAttr != H->C) continue;
        const size_t plane = (size_t)H->H * H->W;

        for (int gy = 0; gy < H->H; ++gy) {
            for (int gx = 0; gx < H->W; ++gx) {
                const size_t sp = (size_t)gy * H->W + gx;
                for (int a = 0; a < nA; ++a) {
                    #define CH(k) pp_head_val(H, (size_t)(a * nAttr + (k)) * plane + sp)

                    float obj = CH(4);
                    if (H->qdata) {
                        const float thr_q = conf_thres / H->scale + (float)H->zp;
                        if ((float)H->qdata[(size_t)(a * nAttr + 4) * plane + sp] < thr_q)
                            continue;
                    } else if (obj <= conf_thres) {
                        continue;
                    }
                    float best_s = 0.0f;
                    int best_c = -1;
                    for (int c = 0; c < nc; ++c) {
                        float s = CH(5 + c);
                        if (s > best_s) { best_s = s; best_c = c; }
                    }
                    if (!out_has_sigmoid) {
                        obj    = sigmoid_f(obj);
                        best_s = sigmoid_f(best_s);
                    }
                    float conf = obj * best_s;
                    if (conf < conf_thres || best_c < 0) continue;

                    float tx = CH(0), ty = CH(1), tw = CH(2), th = CH(3);
                    if (!out_has_sigmoid) {
                        tx = sigmoid_f(tx); ty = sigmoid_f(ty);
                        tw = sigmoid_f(tw); th = sigmoid_f(th);
                    }

                    float aw = anchors[h * 6 + a * 2 + 0];
                    float ah = anchors[h * 6 + a * 2 + 1];

                    float cx = (tx * 2.0f - 0.5f + (float)gx) * (float)H->stride;
                    float cy = (ty * 2.0f - 0.5f + (float)gy) * (float)H->stride;
                    float bw = powf(tw * 2.0f, 2.0f) * aw;
                    float bh = powf(th * 2.0f, 2.0f) * ah;

                    float x1, y1, x2, y2;
                    if (map_back_px(cx, cy, bw, bh, lb, &x1, &y1, &x2, &y2) != 0)
                        continue;
                    if (written < max_out) {
                        out[written].cls  = best_c;
                        out[written].conf = conf;
                        out[written].x1 = (int)lroundf(x1);
                        out[written].y1 = (int)lroundf(y1);
                        out[written].x2 = (int)lroundf(x2);
                        out[written].y2 = (int)lroundf(y2);
                        written++;
                    }
                }
            }
        }
    }
    return written;
    #undef CH
}


int pp_nms(pp_box_t *boxes, int n, float iou_thres, int agnostic,
           pp_box_t *out, int max_out) {
    if (!boxes || n <= 0 || !out || max_out <= 0) return 0;

    for (int i = 1; i < n; ++i) {
        pp_box_t key = boxes[i];
        int j = i - 1;
        while (j >= 0 && boxes[j].conf < key.conf) { boxes[j + 1] = boxes[j]; --j; }
        boxes[j + 1] = key;
    }

    unsigned char *sup = (unsigned char *)calloc((size_t)n, 1);
    if (!sup) return 0;

    int kept = 0;
    for (int i = 0; i < n && kept < max_out; ++i) {
        if (sup[i]) continue;
        out[kept++] = boxes[i];
        for (int j = i + 1; j < n; ++j) {
            if (sup[j]) continue;
            if (!agnostic && boxes[j].cls != boxes[i].cls) continue;
            if (pp_iou(&boxes[i], &boxes[j]) > iou_thres) sup[j] = 1;
        }
    }
    free(sup);
    return kept;
}


const float *pp_yolov5n_default_anchors(void) {
    static const float a[18] = {
        10.f, 13.f,  16.f, 30.f,   33.f,  23.f,
        30.f, 61.f,  62.f, 45.f,   59.f, 119.f,
       116.f, 90.f, 156.f, 198.f, 373.f, 326.f
    };
    return a;
}


int pp_finish(pp_box_t *cand, int n, const pp_cfg_t *cfg,
              pp_box_t *out, int max_out) {
    if (!cand || !cfg || !out) return -1;

    if (cfg->min_box_px > 0) {
        int w = 0;
        for (int i = 0; i < n; ++i) {
            if ((cand[i].x2 - cand[i].x1) >= cfg->min_box_px &&
                (cand[i].y2 - cand[i].y1) >= cfg->min_box_px)
                cand[w++] = cand[i];
        }
        n = w;
    }
    return pp_nms(cand, n, cfg->iou_thres, cfg->agnostic, out, max_out);
}


pp_light_opt_t pp_light_default_opt(void) {
    pp_light_opt_t o;
    o.contain  = 0.80f;
    o.min_w_in = 30;
    o.do_color = 1;
    return o;
}

static float contain_ratio(const pp_box_t *a, const pp_box_t *b) {
    int x1 = a->x1 > b->x1 ? a->x1 : b->x1;
    int y1 = a->y1 > b->y1 ? a->y1 : b->y1;
    int x2 = a->x2 < b->x2 ? a->x2 : b->x2;
    int y2 = a->y2 < b->y2 ? a->y2 : b->y2;
    if (x2 <= x1 || y2 <= y1) return 0.0f;
    float inter = (float)(x2 - x1) * (float)(y2 - y1);
    float ab = (float)((b->x2 - b->x1) * (b->y2 - b->y1));
    return ab > 0.0f ? inter / ab : 0.0f;
}

static float rgb_hue(int r, int g, int b) {
    int mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
    int mn = r < g ? (r < b ? r : b) : (g < b ? g : b);
    int d = mx - mn;
    if (d == 0) return -1.0f;
    float h;
    if (mx == r)      h = 60.0f * (float)(g - b) / (float)d;
    else if (mx == g) h = 60.0f * (float)(b - r) / (float)d + 120.0f;
    else              h = 60.0f * (float)(r - g) / (float)d + 240.0f;
    if (h < 0.0f) h += 360.0f;
    return h;
}

static int vote_light_color(const uint8_t *rgb, int w, int h,
                            int x1, int y1, int x2, int y2) {
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 > w) x2 = w;
    if (y2 > h) y2 = h;
    if (x2 - x1 < 2 || y2 - y1 < 2) return -1;
    int nr = 0, ng = 0, ny = 0, ntot = 0;
    int step = 1;
    while (((x2 - x1) / step) * ((y2 - y1) / step) > 4000) step++;
    for (int y = y1; y < y2; y += step) {
        const uint8_t *p = rgb + ((size_t)y * w + x1) * 3;
        for (int x = x1; x < x2; x += step, p += (size_t)step * 3) {
            int r = p[0], g = p[1], b = p[2];
            int mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
            int mn = r < g ? (r < b ? r : b) : (g < b ? g : b);
            if (mx < 110) continue;
            if (mx - mn < 40) continue;
            float hue = rgb_hue(r, g, b);
            if (hue < 0.0f) continue;
            ntot++;
            if (hue <= 12.0f || hue >= 165.0f) nr++;
            else if (hue >= 38.0f && hue <= 95.0f) ng++;
            else if (hue > 12.0f && hue < 38.0f) ny++;
        }
    }
    if (ntot < 4) return -1;
    int best = nr, k = 2;
    if (ng > best) { best = ng; k = 3; }
    if (ny > best) { best = ny; k = 4; }
    if ((float)best < 0.5f * (float)(nr + ng + ny)) return -1;
    return k;
}

int pp_refine_lights(const uint8_t *rgb, int w, int h, const pp_lb_t *lb,
                     pp_box_t *boxes, int n, const pp_light_opt_t *opt) {
    if (!boxes || n <= 0) return n > 0 ? n : 0;
    pp_light_opt_t o = opt ? *opt : pp_light_default_opt();

    char *drop = (char *)calloc((size_t)n, 1);
    if (!drop) return n;
    for (int i = 0; i < n; ++i) {
        if (boxes[i].cls < 2 || drop[i]) continue;
        for (int j = 0; j < n; ++j) {
            if (j == i || drop[j] || boxes[j].cls < 2) continue;
            if (boxes[j].conf > boxes[i].conf ||
                (boxes[j].conf == boxes[i].conf && j < i)) {
                if (contain_ratio(&boxes[j], &boxes[i]) >= o.contain) { drop[i] = 1; break; }
                if (contain_ratio(&boxes[i], &boxes[j]) >= o.contain) { drop[j] = 1; }
            }
        }
    }

    int m = 0;
    const float ratio = (lb && lb->ratio > 0.0f) ? lb->ratio : 1.0f;
    for (int i = 0; i < n; ++i) {
        if (drop[i]) continue;
        if (boxes[i].cls >= 2) {
            float win = (float)(boxes[i].x2 - boxes[i].x1) * ratio;
            if (o.min_w_in > 0 && win < (float)o.min_w_in) continue;
            if (o.do_color && rgb) {
                int x1 = (int)((float)boxes[i].x1 * ratio) + (lb ? lb->pad_x : 0);
                int y1 = (int)((float)boxes[i].y1 * ratio) + (lb ? lb->pad_y : 0);
                int x2 = (int)((float)boxes[i].x2 * ratio) + (lb ? lb->pad_x : 0);
                int y2 = (int)((float)boxes[i].y2 * ratio) + (lb ? lb->pad_y : 0);
                int k = vote_light_color(rgb, w, h, x1, y1, x2, y2);
                if (k > 0) boxes[i].cls = k;
            }
        }
        if (m != i) boxes[m] = boxes[i];
        m++;
    }
    free(drop);
    return m;
}
