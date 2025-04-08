#include <stdio.h>
#include <zephyr/drivers/ir.h>
#include <zephyr/kernel.h>
#include "trace.h"
#define TX_LEN 100
uint32_t tx_buf[TX_LEN];
uint32_t tx_len = TX_LEN;
uint32_t tx_sent;
const struct device *const ir_dev = DEVICE_DT_GET(DT_NODELABEL(ir));

void ir_tx_cb(const struct device *dev, struct ir_event *evt, void *user_data) {
    tx_sent += evt->data.tx.len;
    if (tx_sent < tx_len) {
        ir_tx(ir_dev, &(tx_buf[tx_sent]), tx_len - tx_sent);
    }
    printf("[%s] dev:%s evt:%d tx_len:%d\n", __func__, dev->name, evt->type, evt->data.tx.len);
}

void ir_rx_cb(const struct device *dev, struct ir_event *evt, void *user_data) {
    if (evt->type == IR_RX_STOPPED) {
        struct ir_event_rx data;
        for (uint32_t i = 0; i < evt->data.rx.len; i++) {
            printf("data[%d] = 0x%x\n", i, evt->data.rx.buf[i]);
        }
        printf("[%s] dev:%s evt:%d rx_len:%d last_data:0x%x\n", __func__, dev->name, evt->type,
               evt->data.rx.len, evt->data.rx.buf[evt->data.rx.len - 1]);
        ir_rx_disable(ir_dev, &data);
    } else {
        for (uint32_t i = 0; i < evt->data.rx.len; i++) {
            printf("data[%d] = 0x%x\n", i, evt->data.rx.buf[i]);
        }
        printf("[%s] dev:%s evt:%d rx_len:%d\n", __func__, dev->name, evt->type, evt->data.rx.len);
    }
}

int main(void) {
    printf("Hello World! %s\n", CONFIG_BOARD);

    tx_buf[0] = 0x80000000 | 0x156; /* 342 about 9ms */
    tx_buf[1] = 0x00000000 | 0xAB;  /* 171 about 4.5ms */
    for (uint16_t i = 2; i < TX_LEN - 1;) {
        tx_buf[i] = 0x80000000 | 0x15;     /* 21  about 565us */
        tx_buf[i + 1] = 0x00000000 | 0x15; /* 21  about 560us */
        i += 2;
    }

    if (TX_LEN & BIT0) {
        tx_buf[TX_LEN - 1] = BIT30 | 0x80000000 | 0x15;
    } else {
        tx_buf[TX_LEN - 1] = BIT30 | 0x00000000 | 0x15;
    }

    ir_set_freq(ir_dev, 38000, 3);
    ir_tx_enable(ir_dev, ir_tx_cb, NULL);
    ir_tx(ir_dev, tx_buf, TX_LEN);

    uint32_t i = 5;
    while (i--) {
        k_sleep(K_MSEC(200));
        ir_tx(ir_dev, tx_buf, TX_LEN);
    }

    ir_set_freq(ir_dev, 40000000, 3);
    ir_rx_enable(ir_dev, ir_rx_cb, NULL, 256, 800000);
    return 0;
}
