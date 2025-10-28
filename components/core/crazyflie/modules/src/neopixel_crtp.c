#include "neopixel_crtp.h"
#include "crtp.h"
#include "neopixel.h"
#include "FreeRTOS.h"
#include "debug_cf.h"
#include <stdio.h>

static void neopixelCrtpCB(CRTPPacket *pk);

void neopixelCrtpInit(void)
{
    crtpRegisterPortCB(CRTP_PORT_NEOPIXEL, neopixelCrtpCB);
}

static void neopixelCrtpCB(CRTPPacket *pk)
{
    DEBUG_PRINT("NeoPixel CRTP packet received: port=0x%02x channel=0x%02x size=%d\n", (pk->header >> 4) & 0x0F, pk->channel, pk->size);
    switch (pk->channel)
    {
    case NEOPIXEL_CHANNEL_SET_PIXEL:
        if (pk->size >= 4)
        { // idx, r, g, b
            uint8_t idx = pk->data[0];
            uint8_t r = pk->data[1];
            uint8_t g = pk->data[2];
            uint8_t b = pk->data[3];
            neopixelSetPixelColor(idx, r, g, b);
        }
        break;

    case NEOPIXEL_CHANNEL_SHOW:
        neopixelShow();
        break;

    case NEOPIXEL_CHANNEL_CLEAR:
        neopixelClear();
        break;

    case NEOPIXEL_CHANNEL_BLINK:
        if (pk->size >= 5)
        { // enable, on_ms_hi, on_ms_lo, off_ms_hi, off_ms_lo
            bool enable = pk->data[0];
            uint16_t on_ms = (pk->data[1] << 8) | pk->data[2];
            uint16_t off_ms = (pk->data[3] << 8) | pk->data[4];
            if (enable)
            {
                neopixelStartBlink(on_ms, off_ms);
            }
            else
            {
                neopixelStopBlink();
            }
        }
        break;

    default:
        break;
    }
}