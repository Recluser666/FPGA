#ifndef UART_TX_H
#define UART_TX_H

#include <stdint.h>
#include <stddef.h>

#include "decision.h"
#include "postproc.h"

#ifdef __cplusplus
extern "C" {
#endif


#define UART_SYNC0       0xA5
#define UART_SYNC1       0x5A
#define UART_HDR_LEN     8
#define UART_BOX_LEN     12
#define UART_CRC_LEN     2

#define UART_PROTO_VER   1

#define UART_F_ALARM     0x01
#define UART_F_TRUNCATED 0x02
#define UART_F_NO_DET    0x04

#define UART_MAX_BOXES   24


typedef struct {
    uint64_t packets;
    uint64_t bytes;
    uint64_t truncated;
    uint64_t write_errors;
    double   last_packet_ms;
    double   kbps;
} uart_stats_t;


typedef struct uart_tx uart_tx_t;

uart_tx_t *uart_tx_open_device(const char *dev, int baud);

uart_tx_t *uart_tx_open_file(const char *path);

void uart_tx_close(uart_tx_t *u);


int uart_tx_packet_size(int n_boxes);

int uart_tx_pack(uint8_t *buf, int buflen,
                 uint16_t seq,
                 const dec_output_t *dec,
                 const pp_box_t *boxes, int n_boxes,
                 int img_w, int img_h);


int uart_tx_send(uart_tx_t *u, uint16_t seq,
                 const dec_output_t *dec,
                 const pp_box_t *boxes, int n_boxes,
                 int img_w, int img_h);

void uart_tx_stats(const uart_tx_t *u, uart_stats_t *out);

#ifdef __cplusplus
}
#endif
#endif
