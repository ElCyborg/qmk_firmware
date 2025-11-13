/* Copyright 2022 Jacky
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#pragma once


//eeconfig
#define EECONFIG_KB_DATA_SIZE 3
// #define EECONFIG_KB_DATA_SIZE  4

#define RGBLIGHT_DISABLE_KEYCODES

#define WS2812_PWM_DRIVER PWMD4
#define WS2812_PWM_CHANNEL 3
#define WS2812_PWM_PAL_MODE 2
#define WS2812_DMA_STREAM STM32_DMA1_STREAM7
#define WS2812_DMA_CHANNEL 7

// Changes done to core, moved here
#define WS2812_TIMING 1645//1645// 1645//1250
#define WS2812_T1H 870//870//870//660 // Width of a 1 bit in ns
#define WS2812_T0H 332 // Width of a 0 bit in ns
#define WS2812_PWM_TARGET_PERIOD 800000
//#undef WS2812_DUTYCYCLE_0
//#define WS2812_DUTYCYCLE_0 (WS2812_PWM_FREQUENCY / (1000000000 / 250))
//#undef WS2812_DUTYCYCLE_1
//#define WS2812_DUTYCYCLE_1 (WS2812_PWM_FREQUENCY / (1000000000 / 850))
// End changes done to core


#define RGB_MATRIX_LED_COUNT 68
#define DYNAMIC_KEYMAP_LAYER_COUNT 8
#define Module_Updata 
#define RGB_MATRIX_KEYPRESSES // reacts to keypresses
//#define RGB_MATRIX_KEYRELEASES         // reacts to keyreleases (instead of keypresses)
#define RGB_MATRIX_FRAMEBUFFER_EFFECTS // enable framebuffer effects
#define RGB_MATRIX_SOLID_REACTIVE_GRADIENT_MODE
#define RGB_MATRIX_TYPING_HEATMAP_SLIM  



// #define RGB_DISABLE_WHEN_USB_SUSPENDED // USB待机时候停止灯效
//#define RGBLIGHT_SLEEP 
// #define RGB_MATRIX_TIMEOUT 0    //rgb矩阵灯自动熄灭时间

//#define RGBLIGHT_DISABLE_KEYCODES

/* UART Config  */
#define SERIAL_DRIVER SD1
// #define SD1_TX_PAL_MODE PAL_MODE_ALTERNATE_OPENDRAIN
#define SD1_TX_PAL_MODE PAL_MODE_ALTERNATE_PUSHPULL  
// #define WS2812_EXTERNAL_PULLUP

/*DEBUG CONFIG*/
// #define DEBUG_MATRIX_SCAN_RATE

#define MOUSEKEY_INTERVAL 5

#define WEAR_LEVELING_LOGICAL_SIZE 8192
#define WEAR_LEVELING_BACKING_SIZE (WEAR_LEVELING_LOGICAL_SIZE * 2)

/*Force NKRO*/


/*DEBOUNCE TIME ms, default 5*/
#define DEBOUNCE 6


#define RGBLIGHT_LAYERS
#define RGBLIGHT_MAX_LAYERS  20

//#define WS2812_EXTERNAL_PULLUP

// #define RGBLIGHT_DISABLE_KEYCODES   //代码函数控制功能
//#define RGB_MATRIX_DEFAULT_VAL 0      //指示灯初始值 0
// #define RGBLIGHT_DEFAULT_MODE  RGBLIGHT_MODE_STATIC_LIGHT	
//#define RGBLIGHT_DEFAULT_MODE RGBLIGHT_EFFECT_BREATHING+1
#define RGB_MATRIX_MAXIMUM_BRIGHTNESS 160
#define RGBLIGHT_DEFAULT_MODE   RGBLIGHT_MODE_RAINBOW_MOOD+2

//#define RGBLIGHT_HUE_STEP 32      //色调      
//#define RGBLIGHT_VAL_STEP 32      //亮度  //步进值，比如说灯，按下fn + ↑ 一次增加32的亮度

#define RGB_MATRIX_HUE_STEP 10
#define RGB_MATRIX_SAT_STEP 10

#define RGB_MATRIX_VAL_STEP 40

#define ENCODER_DEFAULT_POS 0x3

// //20240301
// #define RGB_MATRIX_DEFAULT_MODE (RGB_MATRIX_CYCLE_OUT_IN - 1)
// #define RGB_MATRIX_DEFAULT_VAL      87
  #define RGB_MATRIX_DEFAULT_SPD     128 
  #define RGB_MATRIX_SPD_STEP 64

// #define RGBLIGHT_DEFAULT_MODE 3 


#define RGBLIGHT_LIMIT_VAL     160
#define RGBLIGHT_DEFAULT_VAL   160
#define RGBLIGHT_VAL_STEP      40