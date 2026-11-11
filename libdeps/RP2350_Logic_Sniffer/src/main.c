#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/status_led.h"
#include <tusb.h>
#include <hardware/clocks.h>
#include <hardware/pll.h>
#include "rp2350_sniffer.h"

// Overclock to 250 MHz
static void __not_in_flash_func(reclock)()
{
    // switch clk_sys and clk_peri to pll_usb
    // see code in 2.15.6.1 of the datasheet for useful comments
    clock_configure(clk_sys,
            CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
            CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
            48 * MHZ,
            48 * MHZ);
    clock_configure(clk_peri,
            0,
            CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
            48 * MHZ,
            48 * MHZ);

    // reset PLL
    pll_init(pll_sys, 2, 1500 * MHZ, 6, 1);

    // switch clocks back to pll_sys
    clock_configure(clk_sys,
            CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
            CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
            250 * MHZ,
            250 * MHZ);
    clock_configure(clk_peri,
            0,
            CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
            250 * MHZ,
            250 * MHZ);
}

int main()
{
    reclock();
    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // Configure sniffer pins as inputs with pulldown.
    // Function selection doesn't matter, as the sniffer only reads the pins.
    // It is possible to sniff the output of other peripherals.
    for (int i = 0; i < SNIFFER_PINCOUNT; i++)
    {
        int gpio = SNIFFER_FIRSTPIN + i;
        gpio_init(gpio);
        gpio_set_dir(gpio, false);
        gpio_set_pulls(gpio, false, true);
    }

    // Repeat for ever: wait for USB to be opened, perform capture and reset
    while(true)
    {
        while (!tud_cdc_n_connected(0))
        {
            // Blink slowly when USB serial port is not open
            gpio_put(PICO_DEFAULT_LED_PIN, true);
            sleep_ms(500);
            gpio_put(PICO_DEFAULT_LED_PIN, false);
            sleep_ms(500);
        }

        rp2350_sniffer_start(SNIFFER_PINS_ALL);

        uint32_t last_write = time_us_32();
        while (tud_cdc_n_connected(0))
        {
            // Blink quickly when capture is running
            uint32_t time_now = time_us_32();
            bool led_state = (time_now % 200000) > 100000;
            gpio_put(PICO_DEFAULT_LED_PIN, led_state);

            // Force at least one block every 500ms so that
            // any partial data gets flushed.
            bool force_sync = ((uint32_t)(time_now - last_write) > 500000);

            // Pass captured data to USB
            sniffer_block_t *block = rp2350_sniffer_fetch(force_sync);
            if (block)
            {
                last_write = time_now;
                fwrite(block, sizeof(sniffer_block_t), 1, stdout);
                rp2350_sniffer_release(block);
            }
        }

        // USB closed
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        rp2350_sniffer_stop();
    }
}
