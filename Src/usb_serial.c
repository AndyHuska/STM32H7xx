/*

  usb_serial.c - USB serial port implementation for STM32H7xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2024 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#include "driver.h"

#if USB_SERIAL_CDC

#include "main.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

#ifdef STM32H723xx
extern USBD_HandleTypeDef hUsbDeviceHS;
#else
extern USBD_HandleTypeDef hUsbDeviceFS;
#endif

#include "usb_serial.h"
#include "../grbl/grbl.h"
#include "../grbl/protocol.h"

static stream_rx_buffer_t rxbuf = {0};
static stream_block_tx_buffer2_t txbuf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

volatile usb_linestate_t usb_linestate = {0};

static bool is_connected (void)
{
    return usb_linestate.pin.dtr && hal.get_elapsed_ticks() - usb_linestate.timestamp >= 15;
}

static bool usb_tx_ready (void)
{
#ifdef STM32H723xx
    return is_connected() && hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED && hUsbDeviceHS.pClassData != NULL;
#else
    return is_connected() && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && hUsbDeviceFS.pClassData != NULL;
#endif
}

static bool usb_tx_blocking (void)
{
    return usb_tx_ready() && hal.stream_blocking_callback();
}

static void usb_tx_discard (void)
{
    txbuf.s = txbuf.use_tx2data ? txbuf.data2 : txbuf.data;
    txbuf.length = 0;
}

//
// Returns number of free characters in the input buffer
//
static uint16_t usbRxFree (void)
{
    uint16_t tail = rxbuf.tail, head = rxbuf.head;
    return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the input buffer
//
static void usbRxFlush (void)
{
    rxbuf.head = rxbuf.tail = 0;
}

//
// Flushes and adds a CAN character to the input buffer
//
static void usbRxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head =  BUFNEXT(rxbuf.head, rxbuf);;
}

//
// Writes current buffer to the USB output stream, swaps buffers
//
static inline bool usb_write (void)
{
    static uint8_t dummy = 0;

    if(!usb_tx_ready()) {
        usb_tx_discard();
        return true;
    }

    txbuf.s = txbuf.use_tx2data ? txbuf.data2 : txbuf.data;

#ifdef STM32H723xx
    while(CDC_Transmit_HS((uint8_t *)txbuf.s, txbuf.length) == USBD_BUSY) {
#else
    while(CDC_Transmit_FS((uint8_t *)txbuf.s, txbuf.length) == USBD_BUSY) {
#endif
        if(!usb_tx_blocking()) {
            usb_tx_discard();
            return false;
        }
    }

    if(txbuf.length % 64 == 0) {
#ifdef STM32H723xx
        while(CDC_Transmit_HS(&dummy, 0) == USBD_BUSY) {
#else
        while(CDC_Transmit_FS(&dummy, 0) == USBD_BUSY) {
#endif
            if(!usb_tx_blocking()) {
                usb_tx_discard();
                return false;
            }
        }
    }

    txbuf.use_tx2data = !txbuf.use_tx2data;
    txbuf.s = txbuf.use_tx2data ? txbuf.data2 : txbuf.data;
    txbuf.length = 0;

    return true;
}

//
// Writes a single character to the USB output stream, blocks if buffer full
//
static bool usbPutC (const uint8_t c)
{
    static uint8_t buf[1];

    if(!usb_tx_ready())
        return false;

    *buf = c;

#ifdef STM32H723xx
    while(CDC_Transmit_HS(buf, 1) == USBD_BUSY) {
#else
    while(CDC_Transmit_FS(buf, 1) == USBD_BUSY) {
#endif
        if(!usb_tx_blocking())
            return false;
    }

    return true;
}

//
// Writes a null terminated string to the USB output stream, blocks if buffer full
// Buffers string up to EOL (LF) before transmitting
//
static void usbWriteS (const char *s)
{
    if(!usb_tx_ready()) {
        usb_tx_discard();
        return;
    }

    size_t length = strlen(s);

    if(length == 0)
        return;

    if(txbuf.length && (txbuf.length + length) > txbuf.max_length) {
        if(!usb_write())
            return;
    }

    while(length > txbuf.max_length) {
        txbuf.length = txbuf.max_length;
        memcpy(txbuf.s, s, txbuf.length);
        if(!usb_write())
            return;
        length -= txbuf.max_length;
        s += txbuf.max_length;
    }

    if(length) {
        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;
        if(s[length - 1] == ASCII_LF)
            usb_write();
    }
}

//
// Writes a number of characters from string to the USB output stream, blocks if buffer full
//
static void usbWrite (const uint8_t *s, uint16_t length)
{
    if(!usb_tx_ready()) {
        usb_tx_discard();
        return;
    }

    if(length == 0)
        return;

    if(txbuf.length && (txbuf.length + length) > txbuf.max_length) {
        if(!usb_write())
            return;
    }

    while(length > txbuf.max_length) {
        txbuf.length = txbuf.max_length;
        memcpy(txbuf.s, s, txbuf.length);
        if(!usb_write())
            return;
        length -= txbuf.max_length;
        s += txbuf.max_length;
    }

    if(length) {
        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;
        usb_write();
    }
}
//
// usbGetC - returns -1 if no data available
//
static int32_t usbGetC (void)
{
    uint16_t tail = rxbuf.tail;

    if(tail == rxbuf.head)
        return -1; // no data available else EOF

    int32_t data = (int32_t)rxbuf.data[tail];   // Get next character, increment tmp pointer
    rxbuf.tail = BUFNEXT(tail, rxbuf);          // and update pointer

    return data;
}

static bool usbSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool usbEnqueueRtCommand (uint8_t c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr usbSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

// NOTE: USB interrupt priority should be set lower than stepper/step timer to avoid jitter
// It is set in HAL_PCD_MspInit() in usbd_conf.c
const io_stream_t *usbInit (void)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state.is_usb = On,
        .state.linestate_event = On,
        .is_connected = is_connected,
        .read = usbGetC,
        .write = usbWriteS,
        .write_char = usbPutC,
        .write_n = usbWrite,
        .enqueue_rt_command = usbEnqueueRtCommand,
        .get_rx_buffer_free = usbRxFree,
        .reset_read_buffer = usbRxFlush,
        .cancel_read_buffer = usbRxCancel,
        .suspend_read = usbSuspendInput,
        .set_enqueue_rt_handler = usbSetRtHandler
    };

    MX_USB_DEVICE_Init();

    txbuf.s = txbuf.data;
    txbuf.max_length = BLOCK_TX_BUFFER_SIZE;

    return &stream;
}

// NOTE: A call to this function should be added as the first line of CDC_Receive_FS() & CDC_Receive_HS().
//       These are found in the usbd_cdc_if.c support files for H743 and H723 parts.
void usbBufferInput (uint8_t *data, uint32_t length)
{
    while(length--) {
        if(!enqueue_realtime_command(*data)) {                  // Check and strip realtime commands,
            uint16_t next_head = BUFNEXT(rxbuf.head, rxbuf);    // Get and increment buffer pointer
            if(next_head == rxbuf.tail)                         // If buffer full
                rxbuf.overflow = 1;                             // flag overflow
            else {
                rxbuf.data[rxbuf.head] = *data;                 // if not add data to buffer
                rxbuf.head = next_head;                         // and update pointer
            }
        }
        data++;                                                 // next...
    }
}

#endif
