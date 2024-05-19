#include "knx.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <string.h>

#include "cdcacm.h"

/*
 *
 https://www.mikrocontroller.net/attachment/151008/KNX_Twisted_Pair_Protokollbeschreibung.pdf
 ** Received Telegrams
 *   BC: Control 10111100 ddrdpp00
 *          - ddd=101 --> data request (vs. extended data request, poll request,
                                            ack, nack, busy)
 *          - r=1     --> not repeated
 *          - pp=11   --> normal, low
 *   11
 *   02: Source 0001000100000010: 1.1.2 (GIRA Tastsensor)
 *   00
 *   03: Receiver 0/0/3 (Licht 1 - Schalten)
 *
 *   E1 00 81
 *
 *     NPCI     T/A-PCI  Payload
 *   = 11100001 00000000 10000001
 *     11100001: N_PDU --> Group address, routing counter 110, payload length 1
 *              000000: T_PDU --> UDP
 *                    00 10000001: A_PDU --> APCI=0010 (GroupValueWrite), d=1
 *
 *   E1 00 80
 *   11100001: Group address, routing counter 110, payload length 1
 *      000000: UDP (01 ssss = NDP, sequence s, 1x xxxx = Control packet)
 *       00 10000000: APCI=0010 (GroupValueWrite), d=0
 *
 *  33 / 32: 00110011 / 00110010: Odd parity cross check
 *      10111100
 *      00010001
 *      00000010
 *      00000000
 *      00000011
 *      11100001
 *      00000000
 *      10000001 / 10000000
 *      --------
 *      00110011 / 00110010
 *
 ** Question:
 *  - Send IACKs by U_SetAddress.req or by U_Ackn.req?
 */

#define KNX_RX_BUF_LEN 264
#define KNX_TX_BUF_LEN 64
#define KNX_DECODED_LEN 16

static volatile char __rx_buf[KNX_RX_BUF_LEN];
static volatile int __rx_at;

static struct knx_telegram __rx_decoded_buf[KNX_DECODED_LEN];
static volatile int __rx_decoded_at;
static volatile int __rx_decoded_read;

static volatile char __tx_buf[KNX_TX_BUF_LEN];
static volatile int __tx_at;
static volatile int __tx_written;

static void send_ack(bool nack, bool busy, bool addressed) {
    if (__tx_at >= KNX_TX_BUF_LEN)
        return;
    __tx_buf[__tx_at] = 0b00010 | (nack << 2) | (busy << 1) | addressed;

    /* enable transmit */
    USART_CR1(USART2) |= USART_CR1_TXEIE;
}

static int decode_telegram() {
    /*
     * -2 = error
     * -1 = not complete, receive in progress
     *  0 = decode completed, telegram disregarded (extended, polling or
     *              control)
     *  1 = decode completed, telegram stored
     */
    if (__rx_at == 0)
        return 0;

    if (__rx_decoded_at >= KNX_DECODED_LEN)
        return -2;
    struct knx_telegram *telegram = __rx_decoded_buf + __rx_decoded_at;

    char request_type = (__rx_buf[0] & 0b11000000) >> 6;
    telegram->repeated = !(__rx_buf[0] & 0b00100000);
    telegram->priority = (__rx_buf[0] & 0b00001100) >> 2;

    if (request_type == 0b10) {
        /* Data Request */
        if (__rx_at < 6)
            return -1;
        char payload_length = (__rx_buf[5] & 0b00001111);

        if (__rx_at < payload_length + 8)
            return -1;

        if (__rx_buf[6] & 0b10000000) {
            /* control packet - ignore */
            __rx_at = 0;
            return 0;
        }

        telegram->source_address = (uint16_t)__rx_buf[1] << 8 | __rx_buf[2];
        telegram->target_address = (uint16_t)__rx_buf[3] << 8 | __rx_buf[4];

        telegram->target_is_group_address = __rx_buf[5] & 0b10000000;
        telegram->routing_counter = (__rx_buf[5] & 0b01110000) >> 4;
        telegram->sequence =
            __rx_buf[6] & 0b01000000 ? (__rx_buf[6] & 0b00111100) >> 2 : -1;

        if (payload_length == 0) {
            telegram->apci = (uint16_t)(__rx_buf[6] & 0b00000011) << 8;
        } else {
            telegram->apci = (uint16_t)(__rx_buf[6] & 0b00000011) << 8 |
                             (uint16_t)(__rx_buf[7]);
        }

        telegram->payload_len = payload_length - 1; /* first byte is apci */
        for (int i = 0; i < telegram->payload_len; i++)
            telegram->payload[i] = __rx_buf[8 + i];

        /* check CRC */
        char parity = 0b11111111;
        for (int i = 0; i < __rx_at - 1; i++) {
            parity ^= __rx_buf[i];
        }

        if (parity != __rx_buf[__rx_at - 1]) {
            /* decode error */
            __rx_at = 0;
            return -2;
        }

        /* done */
        __rx_decoded_at++;
        __rx_at = 0;
        return 1;

    } else if (request_type == 0b00) {
        /* Extended Data Request - ignore */
        if (__rx_at < 7)
            return -1;
        int payload_length = __rx_buf[6];
        if (__rx_at < payload_length + 9)
            return -1;

        __rx_at = 0;
        return 0;
    } else if (request_type == 0b11) {
        /* Poll Data Request - ignore */
        if (__rx_at < 7)
            return -1;

        __rx_at = 0;
        return 0;
    }
}

void usart2_isr(void) {
    if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {
        /* RX */
        __rx_buf[__rx_at] = usart_recv(USART2);
        __rx_at++;

        /* Overflow should not happen - just be safe */
        if (__rx_at >= KNX_RX_BUF_LEN)
            __rx_at--;

        if (decode_telegram() == 1) {
            /* TODO: Send ACK only if sensible */
            send_ack(false, false, true);
        }
    }

    if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
        ((USART_SR(USART2) & USART_SR_TXE) != 0)) {
        /* TX */

        if (__tx_written < __tx_at) {
            usart_send(USART2, __tx_buf[__tx_written]);
            __tx_written++;
        }

        if (__tx_written < __tx_at) {
            /* re-enable interrupt */
            USART_CR1(USART2) &= ~USART_CR1_TXEIE;
        } else {
            __tx_written = __tx_at = 0;
        }
    }
}

int knx_init(void) {
    __rx_at = 0;
    __rx_decoded_at = 0;
    __rx_decoded_read = 0;

    __tx_at = 0;
    __tx_written = 0;

    /*
     * TX: PA2
     * RX: PA3
     */
    rcc_periph_clock_enable(RCC_USART2);

    nvic_enable_irq(NVIC_USART2_IRQ);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

    usart_set_baudrate(USART2, 19200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_EVEN);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    /* Enable Receive Interrupt */
    USART_CR1(USART2) |= USART_CR1_RXNEIE;

    usart_enable(USART2);
}

int knx_read(struct knx_telegram *telegram) {
    if (__rx_decoded_at == __rx_decoded_read)
        return 0;

    memcpy(telegram, __rx_decoded_buf + __rx_decoded_read,
           sizeof(struct knx_telegram));

    __rx_decoded_read++;
    if (__rx_decoded_read == __rx_decoded_at)
        __rx_decoded_at = __rx_decoded_read = 0;
    return 1;
}

int knx_write(struct knx_telegram *telegram) {
    /* TODO: Encode into __tx_buf */

    /* enable transmit */
    USART_CR1(USART2) |= USART_CR1_TXEIE;

    return 0;
}

int knx_main(void) { /* nop */
}
