#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "config.h"
#include "delay.h"
#include "i2c.h"
#include "stm32f103/backup.h"
#include "target.h"

#ifndef REG_BOOT
#define REG_BOOT BKP1
#endif

#ifndef CMD_BOOT
#define CMD_BOOT 0x4F42UL
#endif

uint8_t current_status_led = LED_PROG_KEY;


void target_pre_main(void) {
    /* Turn off the LEDS */
    // i2c_led_all_off();

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
    // Setup our i2c controller
    i2c_setup();

    // Set the PROG led to 'red'
    i2c_led_update_status(false);
}

/* This routine comes from libopencm3. But the version we're using doesn't have
 * timeouts. So blocks forever if it can't talk to the keyscanners */

#define I2C_TIMEOUT 1024
static void i2c_write(uint32_t i2c, int addr, uint8_t* data, size_t n) {
    uint16_t timeout = I2C_TIMEOUT;
    while ((I2C_SR2(i2c) & I2C_SR2_BUSY) && timeout--) {
    }

    i2c_send_start(i2c);

    /* Wait for the end of the start condition, master mode selected, and BUSY
     * bit set */
    timeout = I2C_TIMEOUT;
    while (!((I2C_SR1(i2c) & I2C_SR1_SB) && (I2C_SR2(i2c) & I2C_SR2_MSL) &&
             (I2C_SR2(i2c) & I2C_SR2_BUSY)) &&
           timeout--)
        ;
    i2c_send_7bit_address(i2c, addr, I2C_WRITE);

    /* Waiting for address is transferred. */
    timeout = I2C_TIMEOUT;
    while (!(I2C_SR1(i2c) & I2C_SR1_ADDR) && timeout--)
        ;

    /* Clearing ADDR condition sequence. */
    (void)I2C_SR2(i2c);

    for (size_t i = 0; i < n; i++) {
        i2c_send_data(i2c, data[i]);
        timeout = I2C_TIMEOUT;
        while (!(I2C_SR1(i2c) & (I2C_SR1_BTF)) && timeout--)
            ;
    }
    // In libopencm3 i2c_send_stop is in i2c_transfer7. We lifted it here
    // so we don't need all of i2c_transfer7, since we never *read* from i2c in
    // the bootloader
    i2c_send_stop(i2c);
}

void i2c_setup(void) {
    // Enable I2C1 clock
    rcc_periph_clock_enable(RCC_I2C1);
    // rcc_set_i2c_clock_hsi(I2C1);
    i2c_reset(I2C1);
    i2c_peripheral_disable(I2C1);
    uint8_t clock_megahz = 48;
    i2c_set_speed(I2C1, i2c_speed_sm_100k, clock_megahz);
    i2c_peripheral_enable(I2C1);
    /* GPIO for I2C1 */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO6 | GPIO7);
}

#define I2C_LEFT_ADDRESS 0x58
#define TWI_CMD_LED_SET_ALL_TO 0x03
#define TWI_CMD_LED_SET_ONE_TO 0x04

void i2c_led_on(uint8_t led, uint8_t red, uint8_t green, uint8_t blue) {
    uint8_t cmd[5] = {TWI_CMD_LED_SET_ONE_TO, led, blue, green, red};
    i2c_write(I2C1, I2C_LEFT_ADDRESS, cmd, sizeof(cmd));
}

void i2c_led_all_off() {
    uint8_t cmd[4] = {TWI_CMD_LED_SET_ALL_TO, 0x00, 0x00, 0x00};
    i2c_write(I2C1, I2C_LEFT_ADDRESS, cmd, sizeof(cmd));
    return;
}

void i2c_led_update_status(bool status) {
    // Turn off the last LED
    i2c_led_all_off();
    delay_ms(3, 48000000);

    if (status) {
        i2c_led_on(current_status_led, 0x00, 0xff, 0x00);
    } else {
        i2c_led_on(current_status_led, 0xFF, 0x00, 0x00);
    }
    if (++current_status_led > 3) {
        current_status_led = 0;
    }
}
