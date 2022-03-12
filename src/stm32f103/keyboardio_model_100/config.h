/*
 * Copyright (c) 2016, Devan Lai
 *
 * Permission to use, copy, modify, and/or distribute this software
 * for any purpose with or without fee is hereby granted, provided
 * that the above copyright notice and this permission notice
 * appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED


// Without this, dfu-util exits with an error on flashing
#define DFU_WILL_DETACH 0

#define USE_HSI 1
#ifndef APP_BASE_ADDRESS
#define APP_BASE_ADDRESS (0x08000000 + BOOTLOADER_OFFSET)
#endif
#ifndef FLASH_SIZE_OVERRIDE
#define FLASH_SIZE_OVERRIDE 0x100000
#endif
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE  2048
#endif
#ifndef DFU_UPLOAD_AVAILABLE
#define DFU_UPLOAD_AVAILABLE 1
#endif
#ifndef DFU_DOWNLOAD_AVAILABLE
#define DFU_DOWNLOAD_AVAILABLE 1
#endif

#ifndef HAVE_LED
#define HAVE_LED 1
#endif
#ifndef LED_OPEN_DRAIN
#define LED_OPEN_DRAIN 1
#endif
#ifndef LED_GPIO_PORT
#define LED_GPIO_PORT GPIOB
#endif
#ifndef LED_GPIO_PIN
#define LED_GPIO_PIN GPIO0
#endif

#define HAVE_LED 0

#define LED_PROG_KEY 3 // The LED in the top left corner

#ifndef HAVE_BUTTON
#define HAVE_BUTTON 1
#endif
#ifndef BUTTON_ACTIVE_HIGH
#define BUTTON_ACTIVE_HIGH 1
#endif
#ifndef BUTTON_GPIO_PORT
#define BUTTON_GPIO_PORT GPIOB
#endif
#ifndef BUTTON_GPIO_PIN
#define BUTTON_GPIO_PIN GPIO14
#endif
#ifndef BUTTON_USES_PULL
#define BUTTON_USES_PULL 1

#define BUTTON_OUTPUT_GPIO_PORT GPIOB
#define BUTTON_OUTPUT_GPIO_PIN GPIO15


#endif
#ifndef BUTTON_SAMPLE_DELAY_CYCLES
#define BUTTON_SAMPLE_DELAY_CYCLES 1440000
#endif




#ifndef HAVE_USB_PULLUP_CONTROL
#define HAVE_USB_PULLUP_CONTROL 1
#endif

#ifndef USES_GPIOA
#define USES_GPIOA 1
#endif
#ifndef USES_GPIOB
#define USES_GPIOB 1
#endif

#define USES_GPIOC 0

#ifndef USB_PULLUP_GPIO_PORT
#define USB_PULLUP_GPIO_PORT GPIOA
#endif
#ifndef USB_PULLUP_GPIO_PIN
#define USB_PULLUP_GPIO_PIN  GPIO8
#endif
#ifndef USB_PULLUP_ACTIVE_HIGH
#define USB_PULLUP_ACTIVE_HIGH 1
#endif
#ifndef USB_PULLUP_OPEN_DRAIN
#define USB_PULLUP_OPEN_DRAIN 0
#endif

#define USB_VID 0x3496
#define USB_PID 0x0005

#define INNER_STRINGIFY(str) #str
#define STRINGIFY(str) INNER_STRINGIFY(str)

#define LANDING_PAGE_URL "dfu.keyboard.io?vid=" STRINGIFY(USB_VID) ";pid=" STRINGIFY(USB_PID)  ";version=" STRINGIFY(GIT_BUILD_SHA)

#define USB_VENDOR_STRING "Keyboardio"
#define USB_PRODUCT_STRING "Model 100 (Bootloader)"

#endif
