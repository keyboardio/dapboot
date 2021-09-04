# Build for the Keyboardio Model 100 by default
V = 1
GIT_BUILD_SHA := $(shell git rev-parse HEAD |cut -c 1-6)

#ifndef TARGET 
TARGET = KEYBOARDIO_MODEL_100
TARGET_COMMON_DIR = ./stm32f103
TARGET_SPEC_DIR = ./stm32f103/keyboardio_model_100
LDSCRIPT = ./stm32f103/stm32f103x8.ld
ARCH = STM32F1
DEFS += -DGIT_BUILD_SHA=${GIT_BUILD_SHA}
#endif
