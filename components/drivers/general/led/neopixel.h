/**
 * NeoPixel (WS2812B) driver using RMT for ESP-IDF
 *
 * Minimal API to initialize, set pixels, show, clear and blink.
 */
#ifndef __NEOPIXEL_H__
#define __NEOPIXEL_H__

#include <stdint.h>
#include <stdbool.h>

// Initialize the NeoPixel driver on given GPIO and pixel count
void neopixelInit(int gpio_num, int led_count);

// Set pixel color at index (0-based). Color format is RGB (0-255)
void neopixelSetPixelColor(int idx, uint8_t r, uint8_t g, uint8_t b);

// Set all pixels to the same RGB color and update the strip (show).
// This is a convenience helper for setting a static/solid color.
void neopixelSetAllColor(uint8_t r, uint8_t g, uint8_t b);

// Send pixel buffer to the strip
void neopixelShow(void);

// Clear pixel buffer and send (turn off)
void neopixelClear(void);

// Start/stop blinking the current buffer. on_ms/off_ms in milliseconds.
void neopixelStartBlink(uint32_t on_ms, uint32_t off_ms);
void neopixelStopBlink(void);

#endif // __NEOPIXEL_H__
