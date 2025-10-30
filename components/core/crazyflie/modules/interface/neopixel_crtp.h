#ifndef __NEOPIXEL_CRTP_H__
#define __NEOPIXEL_CRTP_H__

#include <stdbool.h>
#include <stdint.h>

// CRTP Port for NeoPixel control (0x09)
#define CRTP_PORT_NEOPIXEL 0x09

// NeoPixel CRTP Channels
#define NEOPIXEL_CHANNEL_SET_PIXEL 0x00 // Set single pixel
#define NEOPIXEL_CHANNEL_SHOW 0x01      // Show current pixels
#define NEOPIXEL_CHANNEL_CLEAR 0x02     // Clear all pixels
#define NEOPIXEL_CHANNEL_BLINK 0x03     // Start/stop blinking
#define NEOPIXEL_BROADCAST_INDEX 0xFF   // Use SET_PIXEL with this index to mean "set all" on firmware

// Command format for SET_PIXEL:
// [pixel_idx(1)] [r(1)] [g(1)] [b(1)]

// Command format for BLINK:
// [enable(1)] [on_ms_hi(1)] [on_ms_lo(1)] [off_ms_hi(1)] [off_ms_lo(1)]

// Command format for SET_ALL (legacy):
// Prefer sending a SET_PIXEL packet with index == NEOPIXEL_BROADCAST_INDEX
// [pixel_idx(1)=0xFF] [r(1)] [g(1)] [b(1)]

void neopixelCrtpInit(void);

#endif // __NEOPIXEL_CRTP_H__