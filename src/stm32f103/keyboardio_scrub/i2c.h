#pragma once
#include <libopencm3/cm3/vector.h>

#include "dapboot.h"
#include "target.h"
#include "config.h"
#include <errno.h>
#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <math.h>


void i2c_setup(void);
void i2c_led_on(uint8_t led, uint8_t red, uint8_t green, uint8_t blue) ;
void i2c_led_off(uint8_t led);
void i2c_led_all_off(void);
void i2c_led_update_status(bool);
