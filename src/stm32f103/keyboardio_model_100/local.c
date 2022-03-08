#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include "target.h"
#include "config.h"
#include "backup.h"
#include "i2c.h"

#ifndef REG_BOOT
#define REG_BOOT BKP1
#endif

#ifndef CMD_BOOT
#define CMD_BOOT 0x4F42UL
#endif


void target_pre_main(void) {
    /* Turn off the LEDS */
    i2c_led_all_off();

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

void target_post_setup (void) {
    i2c_setup();
    i2c_led_on(LED_PROG_KEY, 0xFF);
}       



/* This routine comes from libopencm3. But the version we're using doesn't have timeouts. So blocks forever if it can't talk to the keyscanners */

#define I2C_TIMEOUT 1024
static void i2c_write(uint32_t i2c, int addr, uint8_t *data, size_t n)
{  
    uint16_t timeout = I2C_TIMEOUT;
    while ((I2C_SR2(i2c) & I2C_SR2_BUSY) && timeout--) { 
    }

    i2c_send_start(i2c);

    /* Wait for the end of the start condition, master mode selected, and BUSY bit set */
    timeout = I2C_TIMEOUT;
    while ( !( (I2C_SR1(i2c) & I2C_SR1_SB) && (I2C_SR2(i2c) & I2C_SR2_MSL) && (I2C_SR2(i2c) & I2C_SR2_BUSY) ) && timeout--);
    i2c_send_7bit_address(i2c, addr, I2C_WRITE);

    /* Waiting for address is transferred. */
    timeout = I2C_TIMEOUT;
    while (!(I2C_SR1(i2c) & I2C_SR1_ADDR) && timeout--);

    /* Clearing ADDR condition sequence. */
    (void)I2C_SR2(i2c);

    for (size_t i = 0; i < n; i++) {
        i2c_send_data(i2c, data[i]);
        timeout = I2C_TIMEOUT;
        while (!(I2C_SR1(i2c) & (I2C_SR1_BTF)) && timeout--);
    }
}


void i2c_setup() {
    // Enable I2C1 clock
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_I2C1);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, GPIO9);
    gpio_set(GPIOB, GPIO9);
     //rcc_set_i2c_clock_hsi(I2C1);
    i2c_reset(I2C1);
    i2c_peripheral_disable(I2C1);
    /* HSI is at 8Mhz */
    uint8_t clock_megahz = 32;
    i2c_set_clock_frequency(I2C1, clock_megahz);
    i2c_set_standard_mode(I2C1);
    /* x Mhz / (100kHz * 2) */
    i2c_set_ccr(I2C1, clock_megahz * 5);
    /* Sm mode, (100kHz) freqMhz + 1 */
    i2c_set_trise(I2C1, clock_megahz + 1);

    i2c_peripheral_enable(I2C1);
    /* GPIO for I2C1 */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO6 | GPIO7);

}

#define I2C_LEFT_ADDRESS 0x58
#define TWI_CMD_LED_SET_ALL_TO 0x03
#define TWI_CMD_LED_SET_ONE_TO 0x04

void i2c_led_on(uint8_t led, uint8_t red) {
    uint8_t cmd[5] = {TWI_CMD_LED_SET_ONE_TO, led, 0x00, 0x00, red};
    i2c_write(I2C1, I2C_LEFT_ADDRESS, cmd, 5);
    return;
}

void i2c_led_all_off() {
    uint8_t cmd[4] = {TWI_CMD_LED_SET_ALL_TO, 0x00, 0x00, 0x00};
    i2c_write(I2C1, I2C_LEFT_ADDRESS, cmd, 4);
    return;
    }
/*
uint8_t status_which_led=0;
void i2c_led_update_status() {
    i2c_led_on(status_which_led++, 0xFF);
    i2c_led_all_off();
    if (status_which_led > 7) {
        status_which_led = 0;
    }

}*/
