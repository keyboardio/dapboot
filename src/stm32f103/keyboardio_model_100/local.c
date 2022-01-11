#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include "target.h"
#include "config.h"
#include "backup.h"

#ifndef REG_BOOT
#define REG_BOOT BKP1
#endif

#ifndef CMD_BOOT
#define CMD_BOOT 0x4F42UL
#endif


void target_pre_main(void) {

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

    /* We don't check the RTC backup register. 
     * If the reboot happened, we only care if 
     * they're holding down the 'program' button 
     */
    // uint16_t cmd = backup_read(REG_BOOT);
    // if (cmd == CMD_BOOT) {
    //    force = true;
    // }

    /* Clear the RTC backup register */
    backup_write(REG_BOOT, 0);

#if HAVE_BUTTON
    /* Wait some time in case the button has some debounce capacitor */
    int i;
    for (i = 0; i < BUTTON_SAMPLE_DELAY_CYCLES; i++) {
        __asm__("nop");
    }
    /* Check if the user button is held down */
    if (BUTTON_ACTIVE_HIGH) {
        if (gpio_get(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN)) {
            force = true;
        }
    } else {
        if (!gpio_get(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN)) {
            force = true;
        }
    }
#endif

    return force;
}

