// // /* Copyright 2022 Jacky
// //  *
// //  * This program is free software: you can redistribute it and/or modify
// //  * it under the terms of the GNU General Public License as published by
// //  * the Free Software Foundation, either version 2 of the License, or
// //  * (at your option) any later version.
// //  *
// //  * This program is distributed in the hope that it will be useful,
// //  * but WITHOUT ANY WARRANTY; without even the implied warranty of
// //  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// //  * GNU General Public License for more details.
// //  *
// //  * You should have received a copy of the GNU General Public License
// //  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
// //  */
#include "quantum.h"
#include "uart.h"
#include "smart_ble.h"
#include "keyboard.h"
#include "print.h"
#include "usb_main.h"
#include "usb_util.h"
#include "usb_driver.h"
#include "adc.h"
#include "mk637.h" 
#include "os_detection.h"
#include "raw_hid.h"
#include "process_rgb_matrix.h"
#include "action_util.h"
#include "common.h"
#include "rtc.h"
#include "battery.h"

#ifdef RGB_MATRIX_ENABLE
#    include "rgb_matrix.h"
#endif
#ifdef RGBLIGHT_ENABLE
#    include "rgblight.h"
#endif


// // /************************默认图层定义************************/


// /****************函数声明************************/
extern matrix_row_t raw_matrix[MATRIX_ROWS]; // raw values
/*loop_10Hz函数的定义*/
#define LOOP_10HZ_PERIOD 100
deferred_token loop10hz_token = INVALID_DEFERRED_TOKEN;
uint32_t       loop_10Hz(uint32_t trigger_time, void *cb_arg);
/***********************切换通讯模式***************************/
enum kb_mode_t new_mode;
enum kb_mode_t new_kb_mode;
/***********************外部变量和一些枚举变量***************************/
extern enum kb_mode_t kb_mode;
extern bool           wireless_connected;           /*连接标志位*/
extern bool           suspend;                      /*2.4G休眠标志位*/
extern bool           first_sleep_flag;             /*进入一级休眠标志位*/
bool           interrupt_source_flag;        /*按键中断来源标志位*/
 bool           rgb_matrix_off_flag;          /*关闭矩阵灯标志位*/
 bool           rgblight_off_flag;
 bool           keyboard_no_idle_flag;        /*按键休眠不分连接以解决切模式异常*/
extern uint8_t       last_wireless_mode;           /*用来记忆之前的无线模式*/
extern uint32_t       wireless_no_operation_time;   /*无线模式无操作计时*/
extern uint32_t       first_sleep_time;             /*发送心跳包计时*/
extern uint32_t       packet_send_time;             /*发送心跳包次数*/
extern uint32_t       keyboard_idle_time;           /*按键空闲计时*/
/*battery.h*/
extern enum batt_charge_status_t batt_charge_status;
extern bool                      battery_first_test_flag;   /*一旦重新上电就会重新检测一次adc的标志位*/
extern bool                      ble_connect_flag;          /*蓝牙连接标志位*/
extern bool                      battery_level_report_flag; /*电池待更新标志*/
extern uint8_t                   battery_level;             /*电池电量百分比*/
extern uint8_t                   temp;                      /*电量百分比*/
extern uint8_t                   last_temp;                 /*记忆上次电量百分比*/
bool                             batt_is_charging_flag;
uint32_t                         batt_is_charging_time;
/*蓝牙连接标志位*/
bool     ble_connect_flag  = false;
/****************变量定义************************/
uint8_t blink_count        = 0;
uint8_t blink_period       = 5;
uint8_t blink_rgb_count    = 0;
uint8_t blink_rgb_period   = 2;
uint8_t blink_reset_count  = 0;
uint8_t blink_reset_period = 5;
/***********************模式指示灯***************************/
#define  LONG_PRESS_TIME    3000        // 3000ms
bool     wireless_connect_flag = false; //无线连接标志位
bool     long_press_ble_flag   = false; //长按蓝牙通道标志位
bool     long_press_24G_flag   = false; //长按2.4G通道标志位
uint32_t long_press_ble_time;           //蓝牙通道键按下计时变量
uint32_t long_press_24G_time;           // 2.4G通道键按下计时变量
uint32_t blink_wireless_time;           //正常闪烁计时
uint32_t higher_blink_wireless_time;    //快闪计时
uint32_t ble_connect_time;
uint32_t prf_connect_time;
/***********************自定义功能键的变量***************************/
/*复位*/
uint32_t long_press_reset_time; //按下复位键开始计时变量
uint8_t  blink_reset_time;
uint8_t  mcu_reset_flag;
bool     reset_flag        = false; //恢复出厂标志位
bool     higher_blink_flag = false;
/*锁win*/
bool win_lock_flag        = false;
bool memory_win_lock_flag = false;
/*电量查询*/
bool battery_consult_flag = false;
/*双击切层*/
uint8_t     set_layer_count;
uint32_t    set_layer_time;
/*白灯灯效*/
bool        white_rgb_flag = false;
/***********************休眠***************************/
// #define     RGB_MATRIX_WIRELESS_TIME    60000  // 原始版本
// #define     RGB_MATRIX_WIRELESS_TIME    300000  // 5分钟一级休眠版本
#define     RGB_MATRIX_WIRELESS_TIME    600000  // 10分钟一级休眠版本
#define     PACKET_SEND_TIME            RGB_MATRIX_WIRELESS_TIME/8000+1
/*USB跟随休眠*/
bool     usb_wakeup_send_code_flag;
uint8_t  release_count    = 0;  // usb休眠唤醒添加释放包，防止hold键
uint8_t  usb_suspend_flag = 2;  // usb检测到suspend标志位;0：ACTIVE和在USB模式下;1:SUSPEND和在USB模式下;2:不在0,1的情况；
uint32_t usb_suspend_time;      // usb检测到suspend计时
uint32_t wakeup_count;
/*无线模式超时休眠*/
bool     wakeup_first_sleep_flag      = false;
uint32_t wakeup_first_sleep_time;
/*2.4G跟随休眠*/
bool     sleep_24G_flag = false; /*2.4G进入休眠标志位*/
/*休眠关灯标志位*/
bool     enable_rgb_pin_flag  = false;
uint8_t  rgb_matrix_off_state = 0;


/*
 * Function previously defined in stm32_isr.c
 * Handles wakeup interrupt and setting `exti_flag` when a wakeup occurs that is not a button press?
 * TBD
 */
OSAL_IRQ_HANDLER(VectorE4) {

    OSAL_IRQ_PROLOGUE();


    if( RTC->CRL & RTC_CRL_ALRF )
    {

        RTC->CRL &= ~RTC_CRL_ALRF;
        EXTI->PR = 1<< 17;
        RTC->CRL |= RTC_CRL_CNF;  
        while(!(RTC->CRL & RTC_CRL_RTOFF));
  

        RCC->APB1ENR |= RCC_APB1ENR_BKPEN; 
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        PWR->CR |= PWR_CR_DBP;

        RTC->CRL |= RTC_CRL_CNF;       //Configuration Flag  
        RTC->CNTL = 0x00;   //时间时15s
        RTC->CRL &= ~RTC_CRL_CNF; 
        while(!(RTC->CRL & RTC_CRL_RTOFF));  //RTC operation OFF 


        RTC->CRL &= ~RTC_CRL_CNF; 
    }
  
  
    interrupt_source_flag = true;
//    matrix_init();
    OSAL_IRQ_EPILOGUE();
}

/**
 * Taken from QMK Discord #help channel, convo between Gray and sigprof, msg date 2025/02/24
 * 
 * By using standard implementations, replugging this keyboard will cause a cycle between firmware->bootloader->firmware->bootloader...... 
 * (this also happens when using the mode switch between bt<->usb<->wifi, it would require a double switch either direction to get into the firmware correctly)
 * I assume this is due to the batteries always supplying power to the MCU, if these were not there then after a period the keyboard would always enter firmware first.
 * I tested this earlier, kinda forgot the results, but the below works. w/e
 * 
 * The firmware gotten from the vendor had directly edited STM32F103 board.c and stm32duino.c to change the default behavior to prevent the firmware->bootloader cycle.
 * 
 * Worst case, the stm32duino Maple 003 bootloader supplied from the vendor, and PCB wiring, has ESC as the direct switch pin to boot into the bootloader.  
 */

void board_init(void) {
    // Disable the QMK default behavior to enter the bootloader after any kind of reset.
    BKP->DR10 = RTC_BOOTLOADER_JUST_UPLOADED;
}

//Used by bootmagic and QK_BOOT. 
void bootloader_jump(void) {
    // Enter the bootloader after reset.
    BKP->DR10 = RTC_BOOTLOADER_FLAG;
    NVIC_SystemReset();
}

//void eeconfig_init_kb(void) {
//
//       //防止复位后模式通道记不住，清空之前先读出来
//
//#ifdef MK637_Flash_Store
//        eeconfig_read_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
//    if (variable_data.eeconfig_last_wireless_mode != 0) {
//        last_ble_mode = variable_data.eeconfig_last_wireless_mode;
//    }
//#endif
//
//   #if (EECONFIG_KB_DATA_SIZE) == 0
//    // // Reset Keyboard EEPROM value to blank, rather than to a set value
//    // eeconfig_update_kb(0);
//
//    // test_variable = 10;
//
//
//#endif
//
//#ifdef MK637_Flash_Store
//    variable_data.eeconfig_last_wireless_mode    = last_ble_mode;
//    variable_data.eeconfig_nkro_flag             = 1;  //默认全键无冲
//    variable_data.eeconfig_win_lock_flag =1;
//    // variable_data.eeconfig_encode_toggle =0;
//    eeconfig_update_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
//#endif
//
//
//    eeconfig_init_user(); 
//}


/*全键无冲切换*/
void key_nkro_toggle(void) {
    //全键无冲带记忆
    if (variable_data.eeconfig_nkro_flag != keymap_config.nkro) {
        variable_data.eeconfig_nkro_flag = keymap_config.nkro;
        eeconfig_update_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
    }
}



/*插入检测  PLUG_IN为高电平USB插入;PLUG_IN为低电平USB没插1*/
bool get_plug_mode(void) {
    if (readPin(PLUG_IN))
        return true;
    else
        return false;
}

/*模式切换，带强转功能*/
enum kb_mode_t get_kb_mode(void) {
    if (!gpio_read_pin(BT_MODE)) {
        new_mode = KB_MODE_BLE;
    } else if (!gpio_read_pin(PRF_MODE)) {
        new_mode = KB_MODE_24G;
    } else {
        new_mode = KB_MODE_USB;
    }
    return new_mode;
}


void keyboard_pre_init_kb(void) {
    gpio_set_pin_output(RENUM_PIN);
    gpio_write_pin_high(RENUM_PIN);
    gpio_set_pin_input(BT_MODE);
    gpio_set_pin_input(PRF_MODE);
    gpio_set_pin_input(PLUG_IN);
    //wtf is pin 8
    gpio_set_pin_output(B8);
    gpio_write_pin_low(B8);
}


// //键盘初始化
void keyboard_post_init_kb(void) {
    /*bootloader校验*/
    encode_boot();
    /*复用引脚*/
    AFIO->MAPR = (AFIO->MAPR & ~AFIO_MAPR_SWJ_CFG_Msk);
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_DISABLE;
    adc_init();
    Rtc_Config_Api();
    uart_init(460800);
    wait_ms(450);
    ws2812_init();
    eeconfig_read_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
    higher_blink_flag   = variable_data.eeconfig_higher_blink_flag;
    last_wireless_mode  = variable_data.eeconfig_last_wireless_mode;
    rgb_matrix_off_flag = variable_data.eeconfig_rgb_matrix_off_flag;
    rgblight_off_flag   = variable_data.eeconfig_rgblight_off_flag;
    win_lock_flag       = variable_data.eeconfig_win_lock_flag;
    keymap_config.nkro  = variable_data.eeconfig_nkro_flag;
    eeconfig_update_keymap(&keymap_config);
    /*mac层关闭锁win*/
//    if (eeconfig_read_default_layer() == 8 || eeconfig_read_default_layer() == 16) {
//        if (win_lock_flag == true) {
//            win_lock_flag        = false;
//            memory_win_lock_flag = true;
//        }
//    }
    loop10hz_token = defer_exec(LOOP_10HZ_PERIOD, loop_10Hz, NULL);
}
//1s执行10次 
uint32_t loop_10Hz(uint32_t trigger_time, void *cb_arg)
{
    /*变量计数*/
    blink_count       = (blink_count + 1) % (blink_period * 2);
    blink_rgb_count   = (blink_rgb_count + 1) % (blink_rgb_period *2);
    blink_reset_count = (blink_reset_count + 1) % (blink_reset_period * 2);
    if (blink_reset_count == 0) {
        if (blink_reset_time > 0)   blink_reset_time--;
    }
//    uprintf("blink count: %d, blink rgb: %d, blink reset count: %d, blink reset time: %d", blink_count, blink_rgb_count, blink_reset_count, blink_reset_time);
    /*全键无冲切换*/
    key_nkro_toggle();

    avoid_hold_key_when_usb_wakeup();

    battery_charge_status();

    battery_level_test();         //adc检测
    ble_send_battery();    //蓝牙发送百分比电量
//
////    toggle_layer();
//
//    
    communicate_mode_toggle();
//    
//
    mcu_reset_init();
//    
//            
    wireless_connected_or_disconnected_operate();
//
////       
    sleep_mode();
////
//       
    slow_switch_fast();

    return LOOP_10HZ_PERIOD;
}


/*防止usb唤醒hold键*/
void avoid_hold_key_when_usb_wakeup(void) {
    if(USB_DRIVER.state == USB_ACTIVE){
        if(usb_wakeup_send_code_flag == true){
            tap_code16(KC_F24);
            usb_wakeup_send_code_flag = false;
        }
        if(release_count){
            if(wakeup_count == 0){
                if (keymap_config.nkro == 1) {
                    report_nkro_t send_nkro_report;
                    memset(&send_nkro_report, 0, sizeof(report_nkro_t));
                    host_nkro_send(&send_nkro_report);
                } else {
                    report_keyboard_t send_6nkro_report;
                    memset(&send_6nkro_report, 0, sizeof(report_keyboard_t));
                    host_keyboard_send(&send_6nkro_report);
                }
            }
            wakeup_count++;
            if(wakeup_count == 10){
                wakeup_count = 0;
                release_count--;
            }
        }
    }
}

//void toggle_layer(void){
//    if(timer_elapsed32(set_layer_time) > 200){
//        if(set_layer_count == 2){
//            if(eeconfig_read_default_layer() == 1){
//                set_single_persistent_default_layer(1);
//            }else if (eeconfig_read_default_layer() == 2) {
//                set_single_persistent_default_layer(0);
//            }else if (eeconfig_read_default_layer() == 16) {
//                set_single_persistent_default_layer(5);
//            }else if (eeconfig_read_default_layer() == 32){
//                set_single_persistent_default_layer(4);
//            }
//        }
//        set_layer_count = 0;
//    }
//}
//


/*无线模式断开或连上要改变的一些变量*/
void wireless_connected_or_disconnected_operate(void) {
    /*无线断连*/
    if (!wireless_connected) {
        if (wireless_connect_flag == true) {
            blink_wireless_time   = timer_read32();
            wireless_connect_flag = false;
            ble_connect_flag      = false;
        }
    }
    /*无线连接上操作*/
    if (wireless_connected) {
        if (wireless_connect_flag == false) {
//            uprintf("wireless conenct flag false, %d\n", wireless_connect_flag);
            wireless_no_operation_time = timer_read32();
            blink_period                = 5;
            //发送心跳包变量
            first_sleep_time     = timer_read32();
            packet_send_time      = 0;
            wireless_connect_flag = true;
        }
    }
}

/*恢复出厂设置*/
void mcu_reset_init(void) {
    if (timer_elapsed32(long_press_reset_time) > LONG_PRESS_TIME) {
        if (reset_flag == true) {
            blink_reset_time  = 3;
            blink_reset_count = 0;
            reset_flag        = false;
            mcu_reset_flag    = 1;
        }
        if (blink_reset_time == 0) {
            if (mcu_reset_flag == 1){
                mcu_reset_flag = 2;
            }
            if (mcu_reset_flag == 2) {
                if (blink_period == 2) {
                    higher_blink_flag                        = true;
                    variable_data.eeconfig_higher_blink_flag = higher_blink_flag;
                    eeconfig_update_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
                }
                eeconfig_init();
                mcu_reset();
            }
        }
    }
}

/*慢闪变快闪*/
void slow_switch_fast(void) {
    if (timer_elapsed32(long_press_ble_time) > LONG_PRESS_TIME) {
        if (long_press_ble_flag == true) {
            WIRELESS_PAIR(last_wireless_mode);
            higher_blink_wireless_time = timer_read32();
            blink_period               = 2;
            blink_count                = 0;
            long_press_ble_flag        = false;
        }
    }
    if (timer_elapsed32(long_press_24G_time) > LONG_PRESS_TIME) {
        if (long_press_24G_flag == true) {
            WIRELESS_PAIR(4);
            higher_blink_wireless_time = timer_read32();
            blink_period               = 2;
            blink_count                = 0;
            long_press_24G_flag        = false;
        }
    }
}


/*模式切换*/
void communicate_mode_toggle(void) {
    new_kb_mode = get_kb_mode();
    /*具体进入那个模式*/
    if (kb_mode != new_kb_mode) // ONLY DO IT ONCE WHEN MODE SWITCHED
    {
        kb_mode = new_kb_mode;
        if (higher_blink_flag == false) {
//            uprintf("higher blink flag is false: %d\n", higher_blink_flag);
            if (kb_mode == KB_MODE_BLE) {
                if (last_wireless_mode > 3) {
                    last_wireless_mode = last_wireless_mode & 0x03;
                }
                /*开启蓝牙通道*/
                WIRELESS_START(last_wireless_mode);
                blink_wireless_time = timer_read32();
                /*发送心跳包变量*/
                first_sleep_time = timer_read32();
                packet_send_time       = 0;
            } else if (kb_mode == KB_MODE_24G) {
                WIRELESS_START(4);
                blink_wireless_time = timer_read32(); //慢闪超时计时
//                uprintf("wireless start 1 %d \n", higher_blink_flag);
                /*发送心跳包变量*/
                first_sleep_time = timer_read32();
                packet_send_time  = 0;
            } else if (kb_mode == KB_MODE_USB) {
                WIRELESS_STOP();
            }
            wireless_no_operation_time  = timer_read32(); //无线模式无操作计时
            blink_count                 = 0;
            blink_period                = 5;
        } else {
            if (kb_mode != KB_MODE_USB) {
                if (kb_mode == KB_MODE_BLE) {
                    WIRELESS_PAIR(last_wireless_mode);
                } else {
//                    uprintf("wireless start 2 %d \n", higher_blink_flag);
                    WIRELESS_PAIR(4);
                }
                wireless_no_operation_time = timer_read32(); //无线模式无操作计时
                higher_blink_wireless_time = timer_read32();
                /*发送心跳包变量*/
                first_sleep_time = timer_read32();
                packet_send_time  = 0;
                blink_count       = 0;
                blink_period      = 2;
            } else {
                WIRELESS_STOP();
            }
            higher_blink_flag                        = false;
            variable_data.eeconfig_higher_blink_flag = higher_blink_flag;
            eeconfig_update_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
        }
    }
}




/*caps,num,scroll,win指示灯*/
void host_keyboard_indicator_light_update(void){
    if (kb_mode == KB_MODE_USB || wireless_connected) {
        if (host_keyboard_led_state().caps_lock && 0x02 == 0x02) {
            rgb_matrix_off_state |= 0x01 << 3;
            rgb_matrix_set_color(24, 144, 144, 144);
        }else {
            rgb_matrix_off_state &= ~(0x01 << 3);
        }
        /*win_lock指示灯*/
        if (win_lock_flag == true) {
            rgb_matrix_off_state |= 0x01 << 4;
            rgb_matrix_set_color(21, 144, 144, 144);
        }else {
            rgb_matrix_off_state &= ~(0x01 << 4);
        }
    }
}

/*电量查询*/
void electricity_inquriy(void){
    if(batt_charge_status == batt_no_charge){
        if (battery_consult_flag) {
            rgb_matrix_off_state |= 0x01;
            rgb_matrix_set_color_all(0, 0, 0);
            if(temp <= 30){
                for (uint8_t i = 1; i <= (temp - 1) / 10 + 1; i++) {
                    rgb_matrix_set_color(53+i, 144, 0, 0);
                }
            }else if (temp > 30 && temp <= 70) {
                for (uint8_t i = 1; i <= (temp - 1) / 10 + 1; i++) {
                    rgb_matrix_set_color(53+i, 144, 144, 0);
                }
            }else {
                for (uint8_t i = 1; i <= (temp - 1) / 10 + 1; i++) {
                    rgb_matrix_set_color(53+i, 0, 144, 0);
                }
            }
        }else {
            rgb_matrix_off_state &= ~0x01;
        }
    }else {
        if(battery_consult_flag){
            rgb_matrix_off_state &= ~0x01;
            battery_consult_flag = !battery_consult_flag;
        }
    }
}

/*充电指示灯*/
void charging_indicator_light(void){
    /*充电指示灯*/
    if(batt_charge_status == batt_is_charging){
        if(batt_is_charging_flag == false){
            batt_is_charging_time = timer_read32();
            batt_is_charging_flag = true;
        }
        if(timer_elapsed32(batt_is_charging_time) < 5000){
            rgb_matrix_off_state |= 0x01 << 5;
            rgb_matrix_set_color(0, 0, 144, 0);
        }else{
            rgb_matrix_off_state &= ~(0x01 << 5);
        }
    }else if (batt_charge_status == batt_no_charge) {
        if(batt_is_charging_flag == true){
            batt_is_charging_flag = false;
        }
        if(temp <= 10){
            rgb_matrix_off_state |= 0x01 << 5;
            if (blink_rgb_count > blink_rgb_period) {
                rgb_matrix_set_color(0, 144, 0, 0);
            } else {
                rgb_matrix_set_color(0, 0, 0, 0);
            }
        }else {
            rgb_matrix_off_state &= ~(0x01 << 5);
        }
    }else {
        rgb_matrix_off_state &= ~(0x01 << 5);
    }
}

/*复位指示灯*/
void mcu_reset_indicator_light(void){
    if (blink_reset_time != 0) {
        rgb_matrix_off_state |= 0x01 << 6;
        if (blink_reset_count > blink_reset_period) {
            rgb_matrix_set_color_all(60,60,60);
        } else {
            rgb_matrix_set_color_all(0,0,0);
        }
    }else {
        rgb_matrix_off_state &= ~(0x01 << 6);
    }
}

/*symotion-prefix)休眠关灯*/
void sleep_off_rgb(void){
    if ((batt_charge_status == batt_no_charge && temp == 0) || enable_rgb_pin_flag == false ) {
//        uprintf("batt charge: %d, batt charge: %d, temp: %d, enable rgb pin: %d\n", batt_charge_status, batt_no_charge, temp, enable_rgb_pin_flag);
        rgb_matrix_set_color_all(RGB_OFF);
    }
}

bool rgb_matrix_indicators_advanced_kb(uint8_t led_min, uint8_t led_max) {
     if (rgb_matrix_off_flag == true || (batt_charge_status == batt_no_charge && (temp <= 10))){
         rgb_matrix_set_color_all(RGB_OFF);
     }
     /*电量查询*/
     electricity_inquriy();
     /*对应什么模式会亮什么灯*/
     mode_indicator_light_init();
     /*caps指示*/
     host_keyboard_indicator_light_update();
     /*充电指示灯*/
     charging_indicator_light();
     /*复位指示灯*/
     mcu_reset_indicator_light();
     /*纯白灯效*/
     if(white_rgb_flag){
         rgb_matrix_off_state |= 0x01 << 6;
         rgb_matrix_set_color_all(60,60,60);
     }else {
         rgb_matrix_off_state &= ~(0x01 << 6);
     }

     rgblight_indicators_advanced_kb();

     /*休眠关灯*/
     sleep_off_rgb();

     return false; 

}

/*模式指示灯*/
    void mode_indicator_light_init(void) {
    if (kb_mode == KB_MODE_BLE && !wireless_connected) {
        rgb_matrix_off_state |= 0x01 << 1;
        ble_connect_time = timer_read32();
        switch (last_wireless_mode) {
            case 1:
                if (blink_count > blink_period)
                    rgb_matrix_set_color(51, 0, 0, 144);
                else
                    rgb_matrix_set_color(51, 0, 0, 0);
                break;
            case 2:
                if (blink_count > blink_period)
                    rgb_matrix_set_color(50, 144, 144, 0);
                else
                    rgb_matrix_set_color(50, 0, 0, 0);
                break;
            case 3:
                if (blink_count > blink_period)
                    rgb_matrix_set_color(49, 144, 0, 0);
                else
                    rgb_matrix_set_color(49, 0, 0, 0);
                break;
            default:
                break;
        }
    } else if (kb_mode == KB_MODE_BLE && wireless_connected) {
        if(timer_elapsed32(ble_connect_time) < 3000){
            rgb_matrix_off_state |= 0x01 << 1;
            switch (last_wireless_mode) {
                case 1:
                    rgb_matrix_set_color(51, 0, 0, 144);
                    break;
                case 2:
                    rgb_matrix_set_color(50, 144, 144, 0);
                    break;
                case 3:
                    rgb_matrix_set_color(49, 144, 0, 0);
                    break;
                default:
                    break;
            }
        }else {
            rgb_matrix_off_state &= ~(0x01 << 1);
        }
    }else if (kb_mode == KB_MODE_24G && !wireless_connected) {
        rgb_matrix_off_state |= 0x01 << 2;
        prf_connect_time = timer_read32();
        if (blink_count > blink_period)
            rgb_matrix_set_color(48, 0, 144, 0);
        else
            rgb_matrix_set_color(48, 0, 0, 0);
    } else if (kb_mode == KB_MODE_24G && wireless_connected) {
        if(timer_elapsed32(prf_connect_time) < 3000){
            rgb_matrix_off_state |= 0x01 << 2;
            rgb_matrix_set_color(48, 0, 144, 0);
        }else {
            rgb_matrix_off_state &= ~(0x01 << 2);
        }
        if (sleep_24G_flag == true) {
            tap_code16(KC_F24);
            sleep_24G_flag = false;
        }
    }else {
        rgb_matrix_off_state &= ~(0x03 << 1);
    }
}


void rgblight_indicators_advanced_kb(void) {
        if (rgblight_off_flag == true || (batt_charge_status == batt_no_charge && (temp <= 10))){
            for(uint8_t i = 3; i < 5; i++){
                rgb_matrix_set_color(i,0,0,0);
            }
        }
        if(white_rgb_flag){
            for(uint8_t i = 3; i < 5; i++){
                rgb_matrix_set_color(i,28,28,28);
            }
        }
        /*休眠关灯*/
        if (enable_rgb_pin_flag == false) {
            for(uint8_t i = 3; i < 5; i++){
                rgb_matrix_set_color(i,20,20,20);
            }
        }

        if((rgb_matrix_off_flag && rgblight_off_flag && rgb_matrix_off_state == 0) || (rgb_matrix_get_val() == 0 && rgblight_get_val() == 0 && rgb_matrix_off_state == 0)){
//            uprintf("if statement\n");
            if (enable_rgb_pin_flag) {
//                uprintf("inside if\n");
                gpio_write_pin_low(EN_BACKLIT);
                enable_rgb_pin_flag = !enable_rgb_pin_flag;
            }
        }else {
            if(!enable_rgb_pin_flag){
                if(!wakeup_first_sleep_flag){
                    gpio_write_pin_high(EN_BACKLIT);
                    enable_rgb_pin_flag = !enable_rgb_pin_flag;
                }
            }
        }

}



bool process_record_kb(uint16_t keycode, keyrecord_t* record) {

        /*长按快闪*/
        if(kb_mode == KB_MODE_BLE){
            if ((keycode >= KC_BLE1 && keycode <= KC_BLE3)) {
                if (last_wireless_mode != keycode - KC_24G){
                    if (record->event.pressed) {
                        WIRELESS_START(keycode - KC_24G);
                        blink_wireless_time = timer_read32();
                        long_press_ble_time = timer_read32();
                        long_press_ble_flag = true;
                        blink_period        = 5;
                        blink_count         = 0;
                    }
                }else{
                    if (record->event.pressed) {
                        long_press_ble_time = timer_read32();
                        long_press_ble_flag = true;
                    } else {
                        if (timer_elapsed32(long_press_ble_time) < LONG_PRESS_TIME) {
                            long_press_ble_flag = false;
                        }
                    }
                }
                return false;
            }
        }
        /*2.4G切换*/
        if (kb_mode == KB_MODE_24G){
            if (keycode == KC_24G) {
                if (record->event.pressed) {
                    long_press_24G_flag = true;
                    long_press_24G_time = timer_read32();
                } else {
                    if (timer_elapsed32(long_press_24G_time) < LONG_PRESS_TIME) {
                        long_press_24G_flag = false;
                    }
                }
                return false;
            }
        }
//        /*切mac层*/
//        if(keycode == KC_MAC){
//            if(record->event.pressed){
//                if(eeconfig_read_default_layer() == 1){
//                    set_single_persistent_default_layer(4);
//                }else if (eeconfig_read_default_layer() == 2) {
//                    set_single_persistent_default_layer(5);
//                }
//                if(win_lock_flag == true){
//                    win_lock_flag        = false;
//                    memory_win_lock_flag = true;
//                }
//            }
//            return false;
//        }
//        /*切win层*/
//        if(keycode == KC_WIN){
//            if(record->event.pressed){
//                if(eeconfig_read_default_layer() == 16){
//                    set_single_persistent_default_layer(0);
//                }else if (eeconfig_read_default_layer() == 32) {
//                    set_single_persistent_default_layer(1);
//                }
//                if(memory_win_lock_flag == true){
//                    win_lock_flag        = true;
//                    memory_win_lock_flag = false;
//                }
//            }
//            return false;
//        }
        /*复位*/
        if (keycode == KC_RESET) {
            if (record->event.pressed) {
                long_press_reset_time = timer_read32();
                reset_flag            = true;
            } else {
                if (timer_elapsed32(long_press_reset_time) < 3000) {
                    reset_flag = false;
                }
            }
            return false;
        }
        
        if(keycode == KC_WHITE){
            if(record->event.pressed)
                white_rgb_flag = true;
            else
                white_rgb_flag = false;
        }
        /*开关灯*/
        if((batt_charge_status != batt_no_charge || temp > 10)){
            if (keycode == RM_TOGG && record->event.pressed) {
                if(rgb_matrix_off_flag == false || rgblight_off_flag == false){
                    rgb_matrix_off_flag = true; /*关闭矩阵灯标志位*/
                    rgblight_off_flag   = true;
                }else {
                    rgb_matrix_off_flag = false; /*关闭矩阵灯标志位*/
                    rgblight_off_flag   = false;
                }
                variable_data.eeconfig_rgb_matrix_off_flag = rgb_matrix_off_flag;
                variable_data.eeconfig_rgblight_off_flag   = rgblight_off_flag;
                eeconfig_update_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
                return false;
            }
        }
        /*关灯时候，所有rgb_martix灯光调试快捷键的全部失效*/
        if (rgb_matrix_off_flag == true || (batt_charge_status == batt_no_charge && temp <= 10)) {
            if(keycode == RM_SPDD || keycode == RM_SPDU || keycode == RM_VALD  || keycode == RM_VALU || keycode == RM_SATD
                ||keycode == RM_SATU  || keycode == RM_HUED || keycode == RM_HUEU || keycode == RM_PREV || keycode == RM_NEXT){
                return false;
            }
        }
    
        if(rgblight_off_flag == true || (batt_charge_status == batt_no_charge && temp <= 10)){
            if(keycode == KC_MODE || keycode == RM_VALD || keycode == RM_VALU){
                return false;
            }
        }
    
//        /*MAC层的自定义按键*/
//        if (keycode == KC_Mctl) {
//            if (record->event.pressed)
//                register_code16(KC_MCTL);
//            else
//                unregister_code16(KC_MCTL);
//        }
//        if (keycode == KC_Lpad) {
//            if (record->event.pressed)
//                register_code16(KC_LPAD);
//            else
//                unregister_code16(KC_LPAD);
//        }
//        if (keycode == KC_Lopt) {
//            if (record->event.pressed)
//                register_code16(KC_LOPT);
//            else
//                unregister_code16(KC_LOPT);
//        }
//        if (keycode == KC_Lcmd) {
//            if (record->event.pressed)
//                register_code16(KC_LCMD);
//            else
//                unregister_code16(KC_LCMD);
//        }
//        if (keycode == KC_Rcmd) {
//            if (record->event.pressed)
//                register_code16(KC_RCMD);
//            else
//                unregister_code16(KC_RCMD);
//        }
        /*锁win*/
        if(keycode == WIN_LOCK){
            if (record->event.pressed){
                win_lock_flag = !win_lock_flag;
                variable_data.eeconfig_win_lock_flag = win_lock_flag ;
                eeconfig_update_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
                return false;
            }
        }
        if(keycode == KC_LGUI || keycode == KC_RGUI){
            if (record->event.pressed){
                if(win_lock_flag){
                    return false;
                }
            }
        }
        /*电池电量检测*/
        if(batt_charge_status == batt_no_charge){
            if (keycode == KC_BAT) {
                if (record->event.pressed)
                    battery_consult_flag = true;
                else
                    battery_consult_flag = false;
                return false;
            }
        }
    
//        if(keycode == MO(2) || keycode == MO(3) || keycode == MO(6)||keycode == MO(7)){
//            if(record->event.pressed){
//                if(timer_elapsed32(set_layer_time) < 200){
//                    if(set_layer_count != 0){
//                        set_layer_count++;
//                    }
//                }
//                if(set_layer_count == 0){
//                    set_layer_count = 1;
//                    set_layer_time = timer_read32();
//                }
//            }
//            return true;
//        }
    
        return process_record_user(keycode, record);
}






void variable_init(void){
    blink_count          = 0;
    blink_rgb_count      = 0;
    blink_reset_count    = 0;
    mcu_reset_flag       = 0;
    blink_reset_time     = 0;
    set_layer_count      = 0;
    reset_flag           = false;
    white_rgb_flag       = false;
    long_press_ble_flag  = false;
    long_press_24G_flag  = false;
    enable_rgb_pin_flag  = false;
    battery_consult_flag = false;
}


/*有线休眠*/
void usb_suspend_power_down(void) {
//    uprintf("usb suspend sleep %d, \n", sleep_24G_flag );
    gpio_write_pin_low(EN_BACKLIT);
    cancel_deferred_exec(loop10hz_token);
    /*晶振*/
    palSetLineMode(OSC_IN,PAL_MODE_INPUT_ANALOG);
    palSetLineMode(OSC_OUT,PAL_MODE_INPUT_ANALOG);
    /*设置按键唤醒休眠*/
    set_row_and_col_when_sleep();
    /*拔插USB唤醒*/
    gpio_set_pin_input(PLUG_IN);
    palEnableLineEvent(PLUG_IN, PAL_EVENT_MODE_BOTH_EDGES);
    gpio_set_pin_input(BT_MODE);
    palEnableLineEvent(BT_MODE, PAL_EVENT_MODE_BOTH_EDGES);
    gpio_set_pin_input(PRF_MODE);
    palEnableLineEvent(PRF_MODE,PAL_EVENT_MODE_BOTH_EDGES);
    _pal_lld_enablepadevent(0, 18, PAL_EVENT_MODE_BOTH_EDGES);
    setPinInput(encoder_left);
    setPinInput(encoder_right);
    palEnableLineEvent(encoder_left, PAL_EVENT_MODE_RISING_EDGE);
    palEnableLineEvent(encoder_right, PAL_EVENT_MODE_RISING_EDGE);
    
    ADC1->CR2 &= ~ADC_CR2_ADON;
    PWR->CR |= (1 << 0) | (1 << 10) | (1 << 11) | (3 << 18);
    PWR->CR |= PWR_CR_CWUF | PWR_CR_CSBF;
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    __WFI();
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    key_debounce();
    SCB->SCR   &= ~SCB_SCR_SLEEPDEEP_Msk;
    EXTI->IMR  &= ~(1 << 18);
    EXTI->EMR  &= ~(1 << 18);
    EXTI->RTSR &= ~(1 << 18);
    EXTI->FTSR &= ~(1 << 18);
    EXTI->PR   =   (1 << 18);
    stm32_clock_init();
    clear_keyboard();
    send_keyboard_report();
    usbWakeupHost(&USB_DRIVER); //解决休眠无法唤醒的问题
    restart_usb_driver(&USB_DRIVER);
    init_usb_driver(&USB_DRIVER); // Should not enter SLEEP when USB mode
    matrix_init();
    adc_init();
    release_count    = 2;
    usb_suspend_flag = 2;
    wakeup_count     = 0;
    /*一旦进入休眠，之前的恢复出厂将会失效*/
    variable_init();
    loop10hz_token   = defer_exec(LOOP_10HZ_PERIOD, loop_10Hz, NULL);
}

/*一级休眠*/
void First_Level_Sleep(void) {
//    uprintf("first level sleep %d, \n", sleep_24G_flag );
    gpio_write_pin_low(EN_BACKLIT);
    cancel_deferred_exec(loop10hz_token);
    usb_disconnect(); // Don't enter SLEEP when USB mode
    /*晶振*/
    palSetLineMode(OSC_IN,PAL_MODE_INPUT_ANALOG);
    palSetLineMode(OSC_OUT,PAL_MODE_INPUT_ANALOG);
    /*关闭usb的DP,DN*/
    gpio_write_pin_low(RENUM_PIN);
    gpio_set_pin_input(A11);
    gpio_set_pin_input(A12);
    /*设置按键唤醒休眠*/
    set_row_and_col_when_sleep();
    /*拔插USB唤醒*/
    gpio_set_pin_input(PLUG_IN);
    palEnableLineEvent(PLUG_IN, PAL_EVENT_MODE_BOTH_EDGES);
    gpio_set_pin_input(BT_MODE);
    palEnableLineEvent(BT_MODE, PAL_EVENT_MODE_BOTH_EDGES);
    gpio_set_pin_input(PRF_MODE);
    palEnableLineEvent(PRF_MODE, PAL_EVENT_MODE_BOTH_EDGES);
    
    setPinInput(encoder_left);
    setPinInput(encoder_right);
    palEnableLineEvent(encoder_left, PAL_EVENT_MODE_RISING_EDGE);
    palEnableLineEvent(encoder_right, PAL_EVENT_MODE_RISING_EDGE);
    
    ADC1->CR2 &= ~ADC_CR2_ADON;
    PWR->CR |= (1 << 0) | (1 << 10) | (1 << 11) | (3 << 18);
    /* Clear Wake-up flag */
    PWR->CR |= PWR_CR_CWUF | PWR_CR_CSBF;
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    /* Request Wait For Interrupt */
    __WFI();
    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    /*按键唤醒去抖*/
    key_debounce();
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    /*时钟初始化*/
    stm32_clock_init();
    /*矩阵初始化以及矩阵扫描*/
    matrix_init();
    matrix_scan();
    adc_init();
    /*发送心跳包变量*/
    first_sleep_time = timer_read32();
    packet_send_time  = 0;
    /*电池电量检测*/
    blink_wireless_time         = timer_read32();
    higher_blink_wireless_time  = timer_read32();
    wireless_no_operation_time  = timer_read32();
    /*usb初始化延时*/
    wakeup_first_sleep_flag = true;
    wakeup_first_sleep_time = timer_read32();
    /*一旦进入休眠，之前的恢复出厂将会失效*/
    variable_init();
    loop10hz_token      = defer_exec(LOOP_10HZ_PERIOD, loop_10Hz, NULL);
}


/*二级休眠*/
void Second_Level_Sleep(void) {
//    uprintf("second level sleep %d, \n", sleep_24G_flag );
    gpio_write_pin_low(EN_BACKLIT);
    cancel_deferred_exec(loop10hz_token);
    kb_mode        = KB_MODE_DEFAULT;
    WIRELESS_STOP();
    wait_ms(10);
    usb_disconnect(); // Don't enter SLEEP when USB mode
    /*晶振*/
    palSetLineMode(OSC_IN,PAL_MODE_INPUT_ANALOG);
    palSetLineMode(OSC_OUT,PAL_MODE_INPUT_ANALOG);
    /*USB的DP、DN*/
    gpio_write_pin_low(RENUM_PIN);
    gpio_set_pin_input(A11);
    gpio_set_pin_input(A12);
    /*设置按键唤醒休眠*/
    set_row_and_col_when_sleep();
    /*拔插USB唤醒*/
    gpio_set_pin_input(PLUG_IN);
    palEnableLineEvent(PLUG_IN, PAL_EVENT_MODE_BOTH_EDGES);
    gpio_set_pin_input(BT_MODE);
    palEnableLineEvent(BT_MODE, PAL_EVENT_MODE_BOTH_EDGES);
    gpio_set_pin_input(PRF_MODE);
    palEnableLineEvent(PRF_MODE,PAL_EVENT_MODE_BOTH_EDGES);
    setPinInput(encoder_left);
    setPinInput(encoder_right);
    palEnableLineEvent(encoder_left, PAL_EVENT_MODE_RISING_EDGE);
    palEnableLineEvent(encoder_right, PAL_EVENT_MODE_RISING_EDGE);
    
    ADC1->CR2 &= ~ADC_CR2_ADON;
    /* Clear Wake-up flag */
    PWR->CR |= (1 << 0) | (1 << 10) | (1 << 11) | (3 << 18);
    PWR->CR |= PWR_CR_CWUF | PWR_CR_CSBF;
    /* Set SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    /* Request Wait For Interrupt */
    __WFI();
    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    /*按键唤醒消抖*/
    key_debounce();
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    stm32_clock_init();
    init_usb_driver(&USB_DRIVER); // Should not enter SLEEP when USB mode
    matrix_init();
    adc_init();
    gpio_write_pin_high(RENUM_PIN);
    /*发送心跳包变量*/
    first_sleep_time = timer_read32();
    packet_send_time  = 0;
    /*超时会重新休眠的变量*/
    blink_wireless_time         = timer_read32();
    higher_blink_wireless_time  = timer_read32();
    wireless_no_operation_time  = timer_read32();
    /*一旦进入休眠，之前的恢复出厂将会失效*/
    variable_init();
    loop10hz_token   = defer_exec(LOOP_10HZ_PERIOD, loop_10Hz, NULL);
}



 void sleep_mode(void) {
     
     /*未连接是灯闪烁:快闪60，慢闪20*/
     if (kb_mode != KB_MODE_USB) {
         if (timer_elapsed32(blink_wireless_time) > 20000) {
//             uprintf("sleep mode 1 before, %d, %d\n", blink_period, wireless_connect_flag);
             if ((blink_period == 5) && (wireless_connect_flag == false)) {

//                 uprintf("sleep mode 1 exec");
                 Second_Level_Sleep();
             }
         }
         if (timer_elapsed32(higher_blink_wireless_time) > 60000) {
//             uprintf("sleep mode 2 before, %d, %d\n", blink_period, wireless_connect_flag);
             if ((blink_period == 2) && (wireless_connect_flag == false)) {
//                 uprintf("sleep mode 2 exec");
                 Second_Level_Sleep();
             }
         }
     }
     /*有线跟随休眠*/
     if (USB_DRIVER.state == USB_SUSPENDED && kb_mode == KB_MODE_USB) {
         if (usb_suspend_flag == 0) {
             usb_suspend_time = timer_read32();
             usb_suspend_flag = 1; /*USB现在是SUSPENED的状态*/
         }
     } else if (USB_DRIVER.state == USB_ACTIVE && kb_mode == KB_MODE_USB) {
         usb_suspend_flag = 0;   /*USB是ACTIVE的状态*/
     } else {
         usb_suspend_flag = 2;   /*USB是ACTIVE和SUSPENED之外的状态*/
     }
     if (timer_elapsed32(usb_suspend_time) > 500) {
         if (get_plug_mode() == true && kb_mode == KB_MODE_USB) {
             if (usb_suspend_flag == 1) {
                 usb_suspend_power_down();
                 usb_wakeup_send_code_flag = true;
             }
         }
     }
     /*2.4G跟随休眠*/
     if (suspend == true) {
         Second_Level_Sleep();
         sleep_24G_flag = true;
         suspend        = false;
     }
     /*无线连接上了无操作超时*/
     if (wireless_connected) {
        
         
         /* Send blanks to keep connection 'alive'. Otherwise, after 20s or so the first keypress is not sent. */
         if(timer_elapsed32(first_sleep_time) >= 8000  && (packet_send_time < 15) ) {
             
             first_sleep_time = timer_read32();
             packet_send_time++;
//             uprintf("send blanks %ld \n", packet_send_time);
             for (int i=0;i<5;i++)
             {
                 uart_write(0x00);
             }       

         }
         
         
         if (timer_elapsed32(wireless_no_operation_time) > RGB_MATRIX_WIRELESS_TIME ) {
//             uprintf("wireless\n");
             config_time_alarm();
             First_Level_Sleep();
             exti_stop_config();
             /*如果唤醒源不是按键唤醒，才进去下面*/
             if (interrupt_source_flag) {
                 interrupt_source_flag = false;
                 wakeup_first_sleep_flag   = false;
                 Second_Level_Sleep();
             } else {
                 first_sleep_flag = true;
                 ble_connect_flag = false;
                 gpio_write_pin_high(RENUM_PIN);
             }
         }
     }
     /*一级休眠唤醒0.1s后再初始化usb_driver,初始化过程中有延时，防止吞键*/
     if (timer_elapsed32(wakeup_first_sleep_time) > 100) {

//         uprintf("wakeup sleep, first flag %d\n", wakeup_first_sleep_flag);
         if (wakeup_first_sleep_flag == true) {
//             uprintf("init usb driver %d\n", wakeup_first_sleep_flag);
             init_usb_driver(&USB_DRIVER);
             wakeup_first_sleep_flag = false;
         }
     }
     /*低电量软关机*/
     if ((temp == 0) && (batt_charge_status == batt_no_charge)) {
         Second_Level_Sleep();
     }
 }



/*蓝牙发送电量*/
void ble_send_battery(void) {
    /*没有按键被按下超过一秒钟，keyboard_no_idle_flag这个标志位才会是 fasle*/
    /* false if nothing has been presseed for 1s */
    if (timer_elapsed32(keyboard_idle_time) >= 1000) {
        keyboard_no_idle_flag = false;
    }
    /*蓝牙模式下发送电池电量*/
    if (kb_mode == KB_MODE_BLE && wireless_connected) // no battery info when 24G or connecting
    {
        if (ble_connect_flag == false) /*蓝牙断连后，又重新连上*/
        {
            battery_level_report_flag = true; /*允许发送电池电量标志位*/
            ble_connect_flag          = true;
        }
        if ((battery_level_report_flag == true) && (keyboard_no_idle_flag == false) && (battery_first_test_flag == true)) {
            sc_ble_battary(temp);
            battery_level_report_flag = false;
        }
    }
}






uint8_t  isr_specal_Trig;

/*休眠时需要设置键盘的row和col脚*/
void set_row_and_col_when_sleep(void) {
#if (DIODE_DIRECTION == ROW2COL)
    
    pin_t col_pins[] = MATRIX_COL_PINS;

    for (uint8_t x = 0; x < MATRIX_COLS; x++) {
        pin_t pin;
        pin = col_pins[x];
        if (pin != NO_PIN) {
            setPinOutput(pin);
            gpio_write_pin_low(pin);
        }
    }
    const long unsigned int row_pins[] = MATRIX_ROW_PINS;
    for (uint8_t x = 0; x < MATRIX_ROWS; x++) {
        pin_t pin;
        pin = row_pins[x];
        if (pin != NO_PIN) {
            gpio_set_pin_input_high(pin);
            palEnableLineEvent(pin, PAL_EVENT_MODE_BOTH_EDGES);
        }
    }
#else

    const long unsigned int row_pins[] = MATRIX_ROW_PINS;
    for (uint8_t x = 0; x < MATRIX_ROWS; x++) {
        pin_t pin;
        pin = row_pins[x];
        if (pin != NO_PIN) {
            setPinOutput(pin);
            gpio_write_pin_low(pin);
        }
    }

    const long unsigned int col_pins[] = MATRIX_COL_PINS;
    for (uint8_t x = 0; x < MATRIX_COLS; x++)
    // for(uint8_t x=0; x<9; x++)
    {
        pin_t pin;
        pin = col_pins[x];
        gpio_set_pin_inputHigh(pin);
        palEnableLineEvent(pin, PAL_EVENT_MODE_BOTH_EDGES);
    }
#endif
}

//休眠防抖
/*按键休眠防抖*/
void key_debounce(void) {
    uint8_t                 isr_source[MATRIX_ROWS];
    uint8_t                 i;
    const long unsigned int row_pins[] = MATRIX_ROW_PINS;
//    uprintf("key debounce, %d", interrupt_source_flag);
    //休眠防抖处理,前提是中断源是按键唤醒
    if (interrupt_source_flag) {
//        uprintf("isrflag\n");
        while (1) {
            wait_ms(3);
            //读10次row口的电平
            for (uint8_t j = 0; j < 10; j++) {
                for (i = 0; i < MATRIX_ROWS; i++) {
                    isr_source[i] = readPin(row_pins[i]);
                    if (!isr_source[i]) break;
                }
                if (i < MATRIX_ROWS) {
                    break;
                }
            }
            if (i >= MATRIX_ROWS) {
                set_row_and_col_when_sleep();
                PWR->CR |= (1 << 0) | (1 << 10) | (1 << 11) | (3 << 18);
                PWR->CR |= PWR_CR_CWUF | PWR_CR_CSBF;
                SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
                __WFI();
                SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
            } else {
                break;
            }
        }
    }
    /*关闭0~15号中断线*/
    EXTI->IMR   &= ~0xFFFF;
    EXTI->EMR   &= ~0xFFFF;
    EXTI->RTSR  &= ~0xFFFF;
    EXTI->FTSR  &= ~0xFFFF;
    EXTI->PR    = 0xFFFF;
    interrupt_source_flag = false;
}




#ifdef Module_Updata
bool via_command_kb(uint8_t *data, uint8_t length) {
    static uint16_t packcount = 0;
    uint8_t         modele_updata[66];
    if (data[0] == 0x0a) {
        switch (data[1]) {
            case 0x55: // connect
                for (uint8_t i = 0; i < 60; i++) {
                    uart_write(0x00);
                }
                wait_ms(5);
                modele_updata[0] = 0x55;
                modele_updata[1] = 0x40;
                for (uint8_t i = 2; i < 66; i++) {
                    modele_updata[i] = data[i - 2];
                }
                uart_transmit(modele_updata, sizeof(modele_updata));
                wait_ms(3);
                break;

            case 0x20: // disable EP1
                modele_updata[0] = 0x55;
                modele_updata[1] = 0x40;
                for (uint8_t i = 2; i < 66; i++) {
                    modele_updata[i] = data[i - 2];
                }
                uart_transmit(modele_updata, sizeof(modele_updata));
                wait_ms(3);
                break;

            case 0x00:
                for (uint8_t i = 0; i < 60; i++) {
                    uart_write(0x00);
                }
                wait_ms(5);
                modele_updata[0] = 0x55;
                modele_updata[1] = 0x40;
                for (uint8_t i = 2; i < 66; i++) {
                    modele_updata[i] = data[i - 2];
                }
                uart_transmit(modele_updata, sizeof(modele_updata));
                wait_ms(3);
                break;

            case 0x01: // check version
                modele_updata[0] = 0x55;
                modele_updata[1] = 0x40;
                for (uint8_t i = 2; i < 66; i++) {
                    modele_updata[i] = data[i - 2];
                }
                uart_transmit(modele_updata, sizeof(modele_updata));
                wait_ms(3);
                break;

            case 0x02: // DFU start
                packcount = 0;

                modele_updata[0] = 0x55;
                modele_updata[1] = 0x40;
                for (uint8_t i = 2; i < 30; i++) {
                    modele_updata[i] = data[i - 2];
                }
                uart_transmit(modele_updata, sizeof(modele_updata));
                wait_ms(3);
                break;

            case 0x03:
                packcount        = 0;
                modele_updata[0] = 0x55;
                modele_updata[1] = 0x40;
                for (uint8_t i = 2; i < 66; i++) {
                    modele_updata[i] = data[i - 2];
                }
                uart_transmit(modele_updata, sizeof(modele_updata));
                wait_ms(3);
                break;

            case 0x04:
                modele_updata[0] = 0x55;
                modele_updata[1] = 0x40;
                for (uint8_t i = 2; i < 66; i++) {
                    modele_updata[i] = data[i - 2];
                }
                uart_transmit(modele_updata, sizeof(modele_updata));
                wait_ms(3);
                break;

            case 0x10:
                //打印发下来的数据
                // if( ( (((packcount+1)*62)/256) >   ((packcount*62)/256)  )  || ((packcount+1)*62 >= bin_size)   )
                // {
                //     data[2]=((packcount*62)/256)%256;
                //     data[3]=((packcount*62)/256)/256;
                //     raw_hid_send(data, length);
                // }
                packcount++;
                //透传module数据，增加2个字节
                //在串口接收处拆掉2个字节，上传pc
                modele_updata[0] = 0x55;
                modele_updata[1] = 0x40;
                for (uint8_t i = 2; i < 66; i++) {
                    modele_updata[i] = data[i - 2];
                }
                uart_transmit(modele_updata, sizeof(modele_updata));
                wait_ms(3);
                break;

            default:
                if(data[1] <= 0x40)
                {
                    modele_updata[0] = 0x55;
                    modele_updata[1] = 0x40;
                    for (uint8_t i = 2; i < 66; i++) {
                        modele_updata[i] = data[i - 2];
                    }
                    uart_transmit(modele_updata, sizeof(modele_updata));
                    wait_ms(3);
                    break;
                }
                else
                {
                    return false;
                } 
        }

        return true;
    }
    return false;
}
#endif


