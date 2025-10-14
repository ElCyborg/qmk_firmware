#pragma once

#include_next <board.h>

#undef STM32F103xB
#define STM32F103xE


// Overrides from repo supplied by vendor, redefine here instead of touching core board.h
// TODO: Why these changes in general?  
/*
 * Port B setup.
 * Everything input with pull-up except:
 * PB10    - Push Pull output  (USB switch).
 */
#undef VAL_GPIOBCRL
#undef VAL_GPIOBCRH
#undef VAL_GPIOBODR
#define VAL_GPIOBCRL            0x48888888     /*  PB7...PB0 */
#define VAL_GPIOBCRH            0x08888384     /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFFFFFF

/*
 * Port C setup.
 * Everything input with pull-up except:
 * PC13    - Push Pull output  (LED).
 */
#undef VAL_GPIOCCRL
#undef VAL_GPIOCCRH
#undef VAL_GPIOCODR
#define VAL_GPIOCCRL            0x80884888      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x88384888      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFFFFF

/*
 * Port D setup.
 * Everything input with pull-up except:
 * PD0  - Normal input (XTAL).
 * PD1  - Normal input (XTAL).
 */
#undef VAL_GPIODCRL
#undef VAL_GPIODCRH
#undef VAL_GPIODODR
#define VAL_GPIODCRL            0x88888844      /*  PD7...PD0 */
#define VAL_GPIODCRH            0x88888888      /* PD15...PD8 */
#define VAL_GPIODODR            0xFFFFFFFB