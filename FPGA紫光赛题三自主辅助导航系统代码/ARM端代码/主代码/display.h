#ifndef ADAS_DISPLAY_H
#define ADAS_DISPLAY_H

#include <stdint.h>
#include "postproc.h"

typedef struct display display_t;

display_t *display_open(const char *path, int src_w, int src_h);
void       display_close(display_t *d);

int  display_width(const display_t *d);
int  display_height(const display_t *d);
const char *display_last_error(void);

int display_show(display_t *d, const uint8_t *rgb565,
                 const pp_box_t *boxes, int n,
                 const char *osd);

int display_x_running(void);

#endif
