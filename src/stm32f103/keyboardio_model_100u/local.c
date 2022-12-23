#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "config.h"
#include "delay.h"
#include "stm32f103/backup.h"
#include "target.h"

#ifndef REG_BOOT
#define REG_BOOT BKP1
#endif

#ifndef CMD_BOOT
#define CMD_BOOT 0x4F42UL
#endif



void target_pre_main(void) {
    /* Turn off the LEDS */

    // Fully reset our button gpios so as not to mess with running apps
    gpio_set_mode(BUTTON_GPIO_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, BUTTON_GPIO_PIN);
    gpio_set_mode(BUTTON_OUTPUT_GPIO_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, BUTTON_OUTPUT_GPIO_PIN);

    // Before jumping into user code, try to reset our clock config back to a standard boot config
    RCC_CR   |= 0x00000001;
    RCC_CFGR &= 0xF8FF0000;
    RCC_CR   &= 0xFEF6FFFF;
    RCC_CR   &= 0xFFFBFFFF;
    RCC_CFGR &= 0xFF80FFFF;

    // disable all RCC interrupts
    RCC_CIR   = 0x00000000;

}

bool target_get_force_bootloader(void) {
    bool force = false;

    /* TODO  This code doesn't seem to behave correctly */
    // If we're here because of a reset-button reset, then stay in the bootloader 
    /*
    const uint32_t mask = (RCC_CSR_LPWRRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_IWDGRSTF |
                           RCC_CSR_SFTRSTF | RCC_CSR_PORRSTF);

    bool force = (RCC_CSR & mask) == RCC_CSR_PINRSTF;
    */

    /* Wait some time in case the button has some debounce capacitor */
    for (int i = 0; i < BUTTON_SAMPLE_DELAY_CYCLES; i++) {
        __asm__("nop");
    }
    /* Check if the user button is held down */
    if (gpio_get(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN)) {
        force = true;
    }

    return force;
}

void target_post_setup(void) {


    // Enable the 5V power network
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, GPIO9);
    gpio_clear(GPIOB, GPIO9);

    // Delay for a moment to let the ATTiny88 boot up.
    // With the model 100's initial factory firmware, this works consistently at
    // 76ms but does not work at 75ms. That jibes with the 65ms delay on boot we
    // have the ATTiny88s fused for.
    //
    // We punch it up to 125ms of delay just to be safe.
    delay_ms(125, 48000000);
}

