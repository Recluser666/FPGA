#include "uart_tx.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "frame_proto.h"

struct uart_tx {
    int  fd;
    char name[64];
    int  is_serial;
    uart_stats_t st;
    double t_start;
    uint64_t bytes_window;
};

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static int baud_to_const(int baud) {
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 500000:  return B500000;
        case 921600:  return B921600;
        case 1000000: return B1000000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        default:      return B115200;
    }
}

uart_tx_t *uart_tx_open_device(const char *dev, int baud) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return NULL;

    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd, &tio) != 0) { close(fd); return NULL; }

    cfmakeraw(&tio);
    cfsetispeed(&tio, baud_to_const(baud));
    cfsetospeed(&tio, baud_to_const(baud));
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) { close(fd); return NULL; }
    tcflush(fd, TCIOFLUSH);

    uart_tx_t *u = (uart_tx_t *)calloc(1, sizeof(*u));
    if (!u) { close(fd); return NULL; }
    u->fd = fd;
    u->is_serial = 1;
    snprintf(u->name, sizeof(u->name), "%s@%d", dev, baud);
    u->t_start = now_ms();
    return u;
}

uart_tx_t *uart_tx_open_file(const char *path) {
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) return NULL;
    uart_tx_t *u = (uart_tx_t *)calloc(1, sizeof(*u));
    if (!u) { close(fd); return NULL; }
    u->fd = fd;
    u->is_serial = 0;
    snprintf(u->name, sizeof(u->name), "%s", path);
    u->t_start = now_ms();
    return u;
}

void uart_tx_close(uart_tx_t *u) {
    if (!u) return;
    if (u->fd >= 0) close(u->fd);
    free(u);
}


int uart_tx_packet_size(int n_boxes) {
    if (n_boxes < 0) n_boxes = 0;
    if (n_boxes > UART_MAX_BOXES) n_boxes = UART_MAX_BOXES;
    return UART_HDR_LEN + 10 + n_boxes * UART_BOX_LEN + UART_CRC_LEN;
}

static inline void put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

int uart_tx_pack(uint8_t *buf, int buflen,
                 uint16_t seq,
                 const dec_output_t *dec,
                 const pp_box_t *boxes, int n_boxes,
                 int img_w, int img_h) {
    if (!buf || buflen <= 0 || !dec) return -1;

    int truncated = 0;
    if (n_boxes > UART_MAX_BOXES) { n_boxes = UART_MAX_BOXES; truncated = 1; }
    if (n_boxes < 0) n_boxes = 0;

    int total = uart_tx_packet_size(n_boxes);
    if (total > buflen) return -2;

    uint8_t *p = buf;
    p[0] = UART_SYNC0;
    p[1] = UART_SYNC1;
    put_u16(p + 2, (uint16_t)total);
    put_u16(p + 4, seq);
    p[6] = (uint8_t)n_boxes;

    uint8_t flags = 0;
    if (dec->alarm_active || dec->alarms != ALARM_NONE) flags |= UART_F_ALARM;
    if (truncated) flags |= UART_F_TRUNCATED;
    if (n_boxes == 0) flags |= UART_F_NO_DET;
    p[7] = flags;

    p[8]  = (uint8_t)dec->cmd;
    p[9]  = dec->alarms;
    p[10] = dec->arrow;
    p[11] = dec->leds;
    put_u16(p + 12, (uint16_t)(img_w > 0 ? img_w : 0));
    put_u16(p + 14, (uint16_t)(img_h > 0 ? img_h : 0));
    put_u16(p + 16, 0);

    uint8_t *b = p + UART_HDR_LEN + 10;
    for (int i = 0; i < n_boxes; ++i) {
        const pp_box_t *s = &boxes[i];
        b[0] = (uint8_t)s->cls;
        float cf = s->conf;
        if (cf < 0.0f) cf = 0.0f;
        if (cf > 1.0f) cf = 1.0f;
        b[1] = (uint8_t)(cf * 255.0f + 0.5f);
        put_u16(b + 2, (uint16_t)(s->x1 < 0 ? 0 : s->x1));
        put_u16(b + 4, (uint16_t)(s->y1 < 0 ? 0 : s->y1));
        put_u16(b + 6, (uint16_t)(s->x2 < 0 ? 0 : s->x2));
        put_u16(b + 8, (uint16_t)(s->y2 < 0 ? 0 : s->y2));
        put_u16(b + 10, 0);
        b += UART_BOX_LEN;
    }

    uint16_t crc = frame_crc16_ccitt(buf, (size_t)(total - UART_CRC_LEN));
    buf[total - 2] = (uint8_t)(crc & 0xFF);
    buf[total - 1] = (uint8_t)(crc >> 8);
    return total;
}


static int write_all(int fd, const uint8_t *buf, int len) {
    int done = 0;
    while (done < len) {
        ssize_t w = write(fd, buf + done, (size_t)(len - done));
        if (w < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN) { usleep(200); continue; }
            return -1;
        }
        done += (int)w;
    }
    return done;
}

int uart_tx_send(uart_tx_t *u, uint16_t seq,
                 const dec_output_t *dec,
                 const pp_box_t *boxes, int n_boxes,
                 int img_w, int img_h) {
    if (!u || u->fd < 0) return -1;

    uint8_t buf[UART_HDR_LEN + 10 + UART_MAX_BOXES * UART_BOX_LEN + UART_CRC_LEN];
    int n = uart_tx_pack(buf, (int)sizeof(buf), seq, dec, boxes, n_boxes,
                         img_w, img_h);
    if (n < 0) return -2;

    double t0 = now_ms();
    if (write_all(u->fd, buf, n) < 0) {
        u->st.write_errors++;
        return -3;
    }
    double dt = now_ms() - t0;

    u->st.packets++;
    u->st.bytes += (uint64_t)n;
    u->st.last_packet_ms = dt;
    if (n_boxes > UART_MAX_BOXES) u->st.truncated++;
    u->bytes_window += (uint64_t)n;

    double elapsed = now_ms() - u->t_start;
    if (elapsed >= 1000.0) {
        u->st.kbps = (double)u->bytes_window / 1024.0 / (elapsed / 1000.0);
        u->bytes_window = 0;
        u->t_start = now_ms();
    }
    return n;
}

void uart_tx_stats(const uart_tx_t *u, uart_stats_t *out) {
    if (!u || !out) return;
    *out = u->st;
}
