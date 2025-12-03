/**
 * NeoPixel (WS2812B) driver using RMT for ESP-IDF
 *
 * NOTE: timing values used here match common WS2812B timings and should
 * work on typical ESP32 targets. If you see issues you may need to tune
 * the RMT clock divider/durations for your chip.
 */

#include "neopixel.h"

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "driver/rmt.h"
#include "driver/gpio.h"
#include "esp_err.h"

// RMT settings and timings (for 800kHz WS2812B)
#define RMT_CHANNEL RMT_CHANNEL_0
#define RMT_CLK_DIV 2 // APB 80MHz / 2 = 40MHz -> tick = 25ns

// Ticks for durations (tick = 25ns)
// Typical WS2812 timings: T0H ~350ns, T0L ~800-900ns, T1H ~700ns, T1L ~600ns
#define T0H_TICKS (14) // 14*25ns = 350ns
#define T0L_TICKS (34) // 34*25ns = 850ns
#define T1H_TICKS (28) // 28*25ns = 700ns
#define T1L_TICKS (20) // 20*25ns = 500ns

#define RESET_US 50

static int s_gpio = -1;
static int s_led_count = 0;
static uint8_t *s_buffer = NULL;     // RGB per pixel
static rmt_item32_t *s_items = NULL; // rmt items buffer

static TimerHandle_t s_blink_timer = NULL;
static bool s_blink_state = false;

static void neopixel_build_rmt_items(void)
{
    // Each pixel = 24 bits -> 24 rmt items
    const int bits = 24;
    const int total_items = s_led_count * bits;

    if (s_items)
    {
        free(s_items);
    }
    s_items = calloc(total_items, sizeof(rmt_item32_t));

    if (!s_items)
        return;

    // WS2812 uses GRB order
    for (int px = 0; px < s_led_count; ++px)
    {
        uint8_t g = s_buffer[px * 3 + 1];
        uint8_t r = s_buffer[px * 3 + 0];
        uint8_t b = s_buffer[px * 3 + 2];

        uint32_t word = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;

        for (int bit = 0; bit < bits; ++bit)
        {
            int idx = px * bits + bit;
            // send MSB first
            bool one = (word & (1 << (23 - bit))) != 0;
            if (one)
            {
                s_items[idx].level0 = 1;
                s_items[idx].duration0 = T1H_TICKS;
                s_items[idx].level1 = 0;
                s_items[idx].duration1 = T1L_TICKS;
            }
            else
            {
                s_items[idx].level0 = 1;
                s_items[idx].duration0 = T0H_TICKS;
                s_items[idx].level1 = 0;
                s_items[idx].duration1 = T0L_TICKS;
            }
        }
    }
}

void neopixelInit(int gpio_num, int led_count)
{
    if (led_count <= 0)
        return;

    s_gpio = gpio_num;
    s_led_count = led_count;

    // allocate pixel buffer (RGB)
    s_buffer = calloc(s_led_count * 3, 1);
    if (!s_buffer)
        return;

    // configure RMT
    rmt_config_t config = {
        .rmt_mode = RMT_MODE_TX,
        .channel = RMT_CHANNEL,
        .gpio_num = (gpio_num_t)s_gpio,
        .clk_div = RMT_CLK_DIV,
        .mem_block_num = 1,
        .tx_config = {
            .loop_en = false,
            .carrier_en = false,
            .idle_output_en = true,
            .idle_level = RMT_IDLE_LEVEL_LOW,
        }};

    rmt_config(&config);
    rmt_driver_install(RMT_CHANNEL, 0, 0);

    // build items buffer now (initially zeros -> cleared)
    neopixel_build_rmt_items();
}

void neopixelSetPixelColor(int idx, uint8_t r, uint8_t g, uint8_t b)
{
    if (idx < 0 || idx >= s_led_count || !s_buffer)
        return;
    s_buffer[idx * 3 + 0] = r;
    s_buffer[idx * 3 + 1] = g;
    s_buffer[idx * 3 + 2] = b;
    /* Invalidate the pre-built RMT items so that a subsequent 'show' will
     * rebuild them from the updated buffer. This mirrors behaviour in
     * neopixelSetAllColor which rebuilds/commits immediately.
     */
    if (s_items)
    {
        free(s_items);
        s_items = NULL;
    }
}

// Set all pixels to the same RGB color and immediately show it.
void neopixelSetAllColor(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_buffer || s_led_count <= 0)
        return;

    for (int i = 0; i < s_led_count; ++i)
    {
        s_buffer[i * 3 + 0] = r;
        s_buffer[i * 3 + 1] = g;
        s_buffer[i * 3 + 2] = b;
    }

    if (s_items)
    {
        free(s_items);
        s_items = NULL;
    }
    neopixel_build_rmt_items();
    neopixelShow();
}

void neopixelShow(void)
{
    if (!s_items)
        neopixel_build_rmt_items();
    if (!s_items)
        return;

    const int total_items = s_led_count * 24;

    // Write items
    rmt_write_items(RMT_CHANNEL, s_items, total_items, true);
    // wait for reset time
    ets_delay_us(RESET_US);
}

void neopixelClear(void)
{
    if (!s_buffer)
        return;
    memset(s_buffer, 0, s_led_count * 3);
    if (s_items)
    {
        free(s_items);
        s_items = NULL;
    }
    neopixel_build_rmt_items();
    neopixelShow();
}

static void neopixel_blink_timer_cb(TimerHandle_t xTimer)
{
    uint32_t *p = (uint32_t *)pvTimerGetTimerID(xTimer);
    if (!p)
        return;

    s_blink_state = !s_blink_state;
    if (s_blink_state)
    {
        // show current buffer
        neopixel_build_rmt_items();
        neopixelShow();
        // next period = off
        xTimerChangePeriod(xTimer, pdMS_TO_TICKS(p[1]), 0);
    }
    else
    {
        // clear temporarily
        uint8_t *tmp = calloc(s_led_count * 3, 1);
        if (tmp)
        {
            uint8_t *saved = malloc(s_led_count * 3);
            if (saved)
            {
                memcpy(saved, s_buffer, s_led_count * 3);
                memcpy(s_buffer, tmp, s_led_count * 3);
                free(tmp);
                if (s_items)
                {
                    free(s_items);
                    s_items = NULL;
                }
                neopixel_build_rmt_items();
                neopixelShow();
                memcpy(s_buffer, saved, s_led_count * 3);
                free(saved);
            }
            else
            {
                free(tmp);
            }
        }
        // next period = on
        xTimerChangePeriod(xTimer, pdMS_TO_TICKS(p[0]), 0);
    }
    xTimerStart(xTimer, 0);
}

void neopixelStartBlink(uint32_t on_ms, uint32_t off_ms)
{
    if (s_blink_timer)
    {
        xTimerDelete(s_blink_timer, 0);
        s_blink_timer = NULL;
    }

    // Create periodic timer which toggles; implement as single timer that
    // changes period on each callback. Simpler: use one-shot switching.
    // We'll implement by creating a timer that uses the on_ms initially and
    // on each expiry changes its period to the other value.

    // We will store both intervals in the timer name pointer by packing them
    // into dynamic memory. But for simplicity, create a timer that fires at
    // the smaller of the two and track state; here create a periodic timer
    // with min(on,off) and handle counts is complex. Simpler: create
    // FreeRTOS timers with different periods and swap. To keep code short,
    // just create a timer that uses pdMS_TO_TICKS(on_ms) and on each call
    // change its period to off_ms and vice versa.

    // We'll use the timer's ID pointer to store two uint32_t values: on and off.
    uint32_t *periods = malloc(2 * sizeof(uint32_t));
    if (!periods)
        return;
    periods[0] = on_ms;
    periods[1] = off_ms;

    // create one-shot timer; callback will change period and restart
    s_blink_timer = xTimerCreate("neop_blink", pdMS_TO_TICKS(on_ms), pdFALSE, (void *)periods, neopixel_blink_timer_cb);

    if (s_blink_timer)
    {
        xTimerStart(s_blink_timer, 0);
    }
    else
    {
        free(periods);
    }
}

void neopixelStopBlink(void)
{
    if (s_blink_timer)
    {
        uint32_t *p = (uint32_t *)pvTimerGetTimerID(s_blink_timer);
        xTimerStop(s_blink_timer, 0);
        xTimerDelete(s_blink_timer, 0);
        free(p);
        s_blink_timer = NULL;
    }
    s_blink_state = false;
}
