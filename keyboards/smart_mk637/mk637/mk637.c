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

#ifdef RGB_MATRIX_ENABLE
#    include "rgb_matrix.h"
#endif
#ifdef RGBLIGHT_ENABLE
#    include "rgblight.h"
#endif


// // /************************默认图层定义************************/


// /****************函数声明************************/
extern matrix_row_t raw_matrix[MATRIX_ROWS]; // raw values
 uint32_t exti_flag;
extern uint32_t pr1; 
extern uint8_t                keyboard_led_state;
extern uint8_t ble_led_state ;
extern uint32_t send_packetimer;
extern uint8_t packet_send;
uint8_t temp_test_cout;
extern uint32_t test_isrsource;
extern uint8_t test_variable;
/****************变量定义************************/
//闪烁计时
static uint8_t blink_period = 5;
static uint8_t blink_counter  = 0;
static uint8_t wakeup_pack24G  = 0;
static uint8_t lowpower_flag                    = 1; 
static uint8_t power_period   = 2;
static uint8_t power_counter  = 0;
uint8_t shut_up_Flag = 0 ;
static uint8_t ble24G_period   = 5;
static uint8_t BT_24G_Shine;
static uint8_t  ble_mode; 
static uint8_t  reset_flag; 
static uint8_t  current_layer; 
// bool     sleep_24G_flag = false; // 2.4G进入休眠标志位

bool     usb_wakeup_flag;
// static uint8_t old_effect;
// static uint8_t Default_Effect_Flag;
// static uint8_t  encode_toggle = 0;
// static uint8_t  encode_toggle_flag = 0;
// static uint8_t  encode_Press_flag = 0;

uint8_t  lock_keyboard = 0;
// static uint32_t encode_toggle_timer          = 0;    //旋钮计时
static uint32_t Doubele_Fn_timer          = 0;    //fn计时
static uint32_t First_timer          = 0;    //fn计时
// static uint32_t Power_Full_timer          = 0;    //fn计时
static uint32_t Media_timer          = 0;    //fn计时
uint8_t        sleep_shutled_flag       = 0;
// uint8_t  Power_Full_Flag = 0;
uint8_t  FN_Count = 0;
uint8_t  test_number = 0;
uint8_t  Media_Flag = 0;
 uint8_t Plug_In_Flag;
uint8_t lowpower_effect;
uint8_t chag_via_flag;
uint8_t chag_first_flag;
// static uint8_t Double_Fn_Flag = 0;
uint8_t lock_win_flag = 1;  
static uint8_t Valorspd_counter  = 0;
static uint8_t Valorspd_period   = 3;
static uint8_t blink_num                    = 0;  
//电池待更新标志
static uint8_t battery_Flag = 0;
//按键休眠 不分连接以解决切模式异常
 uint8_t keyboard_Idle = 0;
static uint8_t ble24G_counter  = 0;
static uint8_t temp;
static uint8_t last_temp = 0 ;
static uint8_t second_temp = 0 ;
 uint16_t sleep_switch_driverflag = 0 ;
//充电灯闪烁
/*切换全键无冲变量*/
//static uint8_t BT_Switch_Flag = 1;
uint8_t isr_Trig;
uint8_t nkro_flag = 1;
uint8_t default_layer  = 1;   
extern uint32_t rgb_wireless_timer;
extern uint32_t first_sleep_timer;
 static uint8_t Test_Battery_value;
static uint8_t connect_buff = 0;
bool ble_flag = 0;
static uint32_t ble_bat_timer;
uint8_t Light_Count;
static uint32_t long_ble_24g_timer          = 0;    //蓝牙2.4g长按计时
static uint32_t blink_ble_24g_timer         = 0;    //蓝牙2.4闪烁计时
uint32_t keyborad_Idtimer         = 0;    //按键空闲计时
uint32_t keyborad_Chrgtimer         = 0;    //充电计时
uint32_t encoder_longpresstimer         = 0;    //充电计时
uint32_t test_twoled         = 0;    //按键空闲计时
static uint32_t pair_timeout                = 20000;//20000;    //配对超时时间
static uint32_t pair_succeed_timer          = 0;    //按键无线配对成功灯长亮时间
static uint32_t long_press_reset_timer      = 0;    //记录长按复位键的时
static uint8_t release_count = 0;           //usb休眠唤醒添加释放包，防止hold键
static uint32_t usb_suspend_time;   
// static uint8_t WinLayer_Flag;  
// static uint8_t MacLayer_Flag;  
static uint32_t usb_suspend_time;  
uint8_t test_count  = 0;
/****************标志变量************************/
kb_flag32_t	flag32 = {0}; 
uint8_t usb_suspend_flag = 2;             //usb检测到suspend标志位
//adc检测变量
static uint16_t adc_value = 0;
static uint16_t adc_vref = 0;
uint16_t battery_value = 0;//20240317
bool bat_test_flag = false;
static  uint32_t battery_test_time;
extern bool suspend;
extern bool wireless_connected;
extern uint8_t last_wireless_mode;
 enum kb_mode_t kb_mode= KB_MODE_DEFALT;//每次上电都可以进入下面的上电执行一次的地
extern rgb_led_t leds[6];
#define LOOP_10HZ_PERIOD    100
deferred_token loop10hz_token  = INVALID_DEFERRED_TOKEN;
uint32_t loop_10Hz(uint32_t trigger_time, void *cb_arg);
// Below are used for eeconfig setup
uint32_t last_ble_mode = 1;
uint32_t last_ble24G_period = 0;
uint8_t test_variable = 0;



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
  
  
    exti_flag = 1;
    matrix_init();
    OSAL_IRQ_EPILOGUE();
}

#ifdef RGBLIGHT_ENABLE
const rgblight_segment_t PROGMEM _SecondOff_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {0, 2, HSV_WHITE}
);
const rgblight_segment_t PROGMEM __SecondOn_layer_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {0, 2, HSV_BLACK}
);

const rgblight_segment_t PROGMEM __power1_layer_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {0, 2, HSV_RED}
);



const rgblight_segment_t PROGMEM __power4_layer_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {0, 2, HSV_BLUE}
);
const rgblight_segment_t PROGMEM __power2_layer_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {0, 2, HSV_YELLOW}
);
const rgblight_segment_t PROGMEM __power3_layer_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {0, 2, HSV_RED}
);
const rgblight_segment_t PROGMEM __power5_layer_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {0, 2, HSV_BLACK}
);


const rgblight_segment_t PROGMEM __power6_layer_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {0, 2, HSV_GREEN}
);
const rgblight_segment_t PROGMEM __power7_layer_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {0, 2, HSV_BLACK}
);


const rgblight_segment_t PROGMEM __power8_layer_layer11[] = RGBLIGHT_LAYER_SEGMENTS(
    {0, 2, HSV_GREEN}
);


const rgblight_segment_t PROGMEM __power8_layer_layer12[] = RGBLIGHT_LAYER_SEGMENTS(
    {0, 2, HSV_RED}
);

const rgblight_segment_t PROGMEM __power8_layer_layer13[] = RGBLIGHT_LAYER_SEGMENTS(
    {0, 2, HSV_BLACK}
);


const rgblight_segment_t PROGMEM __power8_layer_layer14[] = RGBLIGHT_LAYER_SEGMENTS(
    {0, 2, HSV_WHITE}
);



const rgblight_segment_t* const PROGMEM my_rgb_layers[] = RGBLIGHT_LAYERS_LIST(
     _SecondOff_layer,
    __SecondOn_layer_layer,
    __power1_layer_layer,

    __power4_layer_layer,
    __power2_layer_layer,
    __power3_layer_layer,
    __power5_layer_layer,
    __power6_layer_layer,
    __power7_layer_layer,
    __power8_layer_layer11,
    __power8_layer_layer12,
    __power8_layer_layer13,
    __power8_layer_layer14
);

#endif

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

void eeconfig_init_kb(void) {
   

       //防止复位后模式通道记不住，清空之前先读出来

#ifdef MK637_Flash_Store
        eeconfig_read_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
    if (variable_data.eeconfig_last_wireless_mode != 0) {
        last_ble_mode = variable_data.eeconfig_last_wireless_mode;
    }
#endif

#ifdef MK856_Flash_Store
        eeconfig_read_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
    if (variable_data.eeconfig_last_wireless_mode != 0) {
        last_ble_mode = variable_data.eeconfig_last_wireless_mode;
    }
#endif

#ifdef MK923_Flash_Store
        eeconfig_read_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
    if (variable_data.eeconfig_last_wireless_mode != 0) {
        last_ble_mode = variable_data.eeconfig_last_wireless_mode;
    }
#endif


#ifdef BK100_Flash_Store
        eeconfig_read_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
    if (variable_data.eeconfig_last_wireless_mode != 0) {
        last_ble_mode = variable_data.eeconfig_last_wireless_mode;
    }
#endif

#ifdef K7214b_Flash_Store
        eeconfig_read_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
    if (variable_data.eeconfig_last_wireless_mode != 0) {
        last_ble_mode = variable_data.eeconfig_last_wireless_mode;
        last_ble24G_period =  variable_data.eeconfig_ble24G_pair_flag;
    }
#endif 
    
   #if (EECONFIG_KB_DATA_SIZE) == 0
    // // Reset Keyboard EEPROM value to blank, rather than to a set value
    // eeconfig_update_kb(0);

    // test_variable = 10;


#endif

#ifdef MK637_Flash_Store
    variable_data.eeconfig_last_wireless_mode    = last_ble_mode;
    variable_data.eeconfig_nkro_flag             = 1;  //默认全键无冲
    variable_data.eeconfig_lock_win_flag =1;
    // variable_data.eeconfig_encode_toggle =0;
    eeconfig_update_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
#endif


#ifdef MK856_Flash_Store
    variable_data.eeconfig_last_wireless_mode    = last_ble_mode;
    variable_data.eeconfig_nkro_flag             = 1;  //默认全键无冲
    variable_data.eeconfig_lock_win_flag =1;
    variable_data.eeconfig_encode_toggle =0;
    variable_data.eeconfig_shut_up_Flag =0;
    variable_data.eeconfig_shut_up_screen_Flag =0;
    eeconfig_update_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
#endif

#ifdef MK923_Flash_Store
    variable_data.eeconfig_last_wireless_mode    = last_ble_mode;
    variable_data.eeconfig_nkro_flag             = 1;  //默认全键无冲
    variable_data.eeconfig_lock_win_flag =1;
    variable_data.eeconfig_encode_toggle =0;
    variable_data.eeconfig_shut_up_Flag =0;
    variable_data.eeconfig_shut_up_screen_Flag =0;
    eeconfig_update_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
#endif


#ifdef K7214b_Flash_Store
    variable_data.eeconfig_last_wireless_mode    = last_ble_mode;
    variable_data.eeconfig_ble24G_pair_flag  = last_ble24G_period;
    variable_data.eeconfig_nkro_flag             = 1;  //默认全键无冲
    variable_data.eeconfig_lock_win_flag =1;
    variable_data.eeconfig_shut_up_Flag =0;
    eeconfig_update_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
#endif

#ifdef BK100_Flash_Store
    variable_data.eeconfig_last_wireless_mode    = last_ble_mode;
    variable_data.eeconfig_nkro_flag             = 1;  //默认全键无冲
    variable_data.eeconfig_lock_win_flag =1;
    variable_data.eeconfig_shut_up_Flag =0;
    eeconfig_update_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
#endif

    eeconfig_init_user(); 
}


/*全键无冲切换*/
void key_nkro_toggle(void) {
    //全键无冲带记忆
    if (nkro_flag != keymap_config.nkro) {
        nkro_flag = keymap_config.nkro;
        // dynamic_keymap_set_keycode(1, 1, 0, nkro_flag);
        variable_data.eeconfig_nkro_flag = keymap_config.nkro;
        eeconfig_update_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);
    }
}


//插入检测  
bool get_plug_mode(void) 
{
    if (readPin(PLUG_IN))
    {
        return true;
    }
    else
    {
        return false;
    }
}

//增加限制
uint8_t count_ble;
uint8_t count_24G;
uint8_t count_USB;
void get_mode(void)
{
    if (!readPin(BLE) )
    {
        kb_mode = KB_MODE_BLE;
       
    }
    else if((!readPin(TwoMode)))
    {
          kb_mode = KB_MODE_24G;
    }
    else 
    {
          kb_mode = KB_MODE_USB;
  
    }

    // if (!readPin(SYS_SW) && (WinLayer_Flag == 0))
    // {
    //     WinLayer_Flag = 1;
    //     MacLayer_Flag = 0;
    //     set_single_persistent_default_layer(2);
    //   //  uprintf("win\r\n");
    // }

    // if (readPin(SYS_SW) && (MacLayer_Flag == 0))
    // {
    //     MacLayer_Flag = 1;
    //     WinLayer_Flag = 0;
    //     set_single_persistent_default_layer(0);
    //     //uprintf("mac\r\n");
    // }
}


#ifdef  mk637_special_isr
void special_key_debounce(void)
{
   //中断源1 左旋    2 2.4G  3蓝牙
   uint8_t temp_status;     

   if(isr_specal_Trig)
   {     
         if(isr_specal_Trig == 1)
         {
              temp_status =  readPin(encoder_left); 
         }
         else if(isr_specal_Trig == 2)
         {
              temp_status =  readPin(TwoMode); 
         }
        else if(isr_specal_Trig == 3)
         {
              temp_status =  readPin(BLE); 
         }
         
        wait_ms(5); 
        if( (isr_specal_Trig == 1 && (temp_status !=  readPin(encoder_left)))  ||  (isr_specal_Trig == 2 && (temp_status !=  readPin(TwoMode)))  || (isr_specal_Trig == 3 && (temp_status !=  readPin(BLE)))  )
        {   
            //中断源再加进去

            setPinInput(BLE);
            setPinInput(TwoMode);
            palEnableLineEvent(BLE, PAL_EVENT_MODE_BOTH_EDGES);
            palEnableLineEvent(TwoMode, PAL_EVENT_MODE_BOTH_EDGES);


            setPinInput(encoder_left);
            setPinInput(encoder_right);
            palEnableLineEvent(encoder_left, PAL_EVENT_MODE_RISING_EDGE);
            palEnableLineEvent(encoder_right, PAL_EVENT_MODE_RISING_EDGE);

            const long unsigned int row_pins[] = MATRIX_ROW_PINS;
            pin_t col_pins[] = MATRIX_COL_PINS;
                for(uint8_t x=0; x<MATRIX_COLS; x++)
                {
                    pin_t pin;
                    pin = col_pins[x];
                    if (pin != NO_PIN) 
                    {
                        setPinOutput(pin);
                        writePinLow(pin);
                    }
                }

             
                for(uint8_t x=0; x<MATRIX_ROWS; x++)
                { 
                    pin_t pin;
                    pin = row_pins[x];
                    if (pin != NO_PIN) 
                    {
                        setPinInputHigh(pin);
                        palEnableLineEvent(pin, PAL_EVENT_MODE_FALLING_EDGE);
                    }
                }
                PWR->CR |= (1<<0)|(1<< 10) |(1<< 11)|(3<<18);
                PWR->CR |= PWR_CR_CWUF | PWR_CR_CSBF;
                SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
                __WFI();
                SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
        }
   }
    isr_specal_Trig = 0;
    EXTI->IMR  &= ~0xffff;   
    EXTI->EMR  &= ~0xffff;
    EXTI->RTSR &= ~0xffff;
    EXTI->FTSR &= ~0xffff;

  
}
#endif
layer_state_t default_layer_state_set_kb(layer_state_t state)
{
    default_layer = state;   
    return state;
}



void keyboard_pre_init_kb(void) {
    setPinOutput(RENUM);
    writePinHigh(RENUM);
    setPinOutput(B8);
    writePinLow(B8);
}


// //键盘初始化
void keyboard_post_init_kb(void)
{
    uart_init(460800);
    wait_ms(400);
    // debug_enable=true;
    // debug_keyboard=true;
    // debug_matrix=true;
    // debug_mouse=true;
    rgblight_layers = my_rgb_layers;
    ws2812_init();
    // setPinOutput(ARGB_LEFT_EN);
    // writePinHigh(ARGB_LEFT_EN);
    AFIO->MAPR = (AFIO->MAPR & ~AFIO_MAPR_SWJ_CFG_Msk);
    AFIO->MAPR|= AFIO_MAPR_SWJ_CFG_DISABLE;
// //全键无冲带记忆
    // nkro_flag       = dynamic_keymap_get_keycode(1, 1, 0);
    // keymap_config.nkro = nkro_flag;
     eeconfig_read_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);    //读出数据  
    keymap_config.nkro  = variable_data.eeconfig_nkro_flag; /*是否开启全全键无冲*/
    // lock_win_flag = variable_data.eeconfig_lock_win_flag;
    // encode_toggle = variable_data.eeconfig_encode_toggle;
    last_wireless_mode  = variable_data.eeconfig_last_wireless_mode;
    eeconfig_update_keymap(&keymap_config);

    // //虚拟按键
    // encode_toggle = dynamic_keymap_get_keycode(0, 4, 9);
    setPinInput(PLUG_IN);
    wait_ms(1); 
    wireless_connected=false;   
    // last_wireless_mode = eeconfig_read_kb();
    // lock_win_flag  = dynamic_keymap_get_keycode(1, 2, 0);
    flag32.mode_one = 1;
    pair_succeed_timer = timer_read32();
//ADC初始化
    adc_init();     //adc初始化
    get_adc_value();
    get_adc_vref();
    Rtc_Config_Api();
    loop10hz_token = defer_exec(LOOP_10HZ_PERIOD, loop_10Hz, NULL);
}


uint8_t limit_flag;
//1s执行10次 
enum kb_mode_t new_kb_mode = KB_MODE_DEFALT;
uint32_t loop_10Hz(uint32_t trigger_time, void *cb_arg)
{
    blink_counter = (blink_counter+1) % (blink_period*2);
    ble24G_counter = (ble24G_counter+1) % (ble24G_period*2);
    power_counter = (power_counter+1) % (power_period*2);  
    Valorspd_counter = (Valorspd_counter+1) % (Valorspd_period*2);    
    if(USB_DRIVER.state == USB_ACTIVE)
    {
        if(usb_wakeup_flag == true){
            clear_keyboard();
            send_keyboard_report();
            usb_wakeup_flag = false;
        }
    }
                
    if(0 == blink_counter)
    {   

          if (release_count) {
            if (USB_DRIVER.state == USB_ACTIVE) {
                release_count--;
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
        }      
    }

    if(!default_layer)
    {
       default_layer = 1; 
    }

    if(timer_elapsed32(keyborad_Idtimer) >= 2000)
    {

        keyboard_Idle = 0;
    }

     if(0 == Valorspd_counter)
     {
      
        if(blink_num > 0)
        {
            blink_num--;
        }

     }
    if(!limit_flag)
    {       
        limit_flag = 1;
        setPinOutput(ARGB_LEFT_EN);
        writePinHigh(ARGB_LEFT_EN);
    }
    

        
    if(lowpower_flag)
    lowpower_flag++;
 
    if((lowpower_flag > 10))
    {
        lowpower_flag = 0;
    }
    /*全键无冲切换*/
    key_nkro_toggle();

    // if(encode_Press_flag)
    // {
    //     if(timer_elapsed32(encode_toggle_timer) > 2000)
    //     {
    //         encode_toggle  = !encode_toggle;
    //         //存储功能
    //         // dynamic_keymap_set_keycode(0, 4, 9, encode_toggle);//虚拟按键
    //         variable_data.eeconfig_encode_toggle = encode_toggle;
    //         //增加常亮1s白色计时
    //         if(!BT_24G_Shine)
    //         {
    //             encode_toggle_flag = 1;
    //             encoder_longpresstimer = timer_read32(); 
    //         }
    //         eeconfig_update_kb_datablock(&variable_data);
    //         encode_Press_flag = 0;
    //     }
    // }


//    if(sleep_switch_driverflag) 
//    {
//         sleep_switch_driverflag = sleep_switch_driverflag -1;
//         if(sleep_switch_driverflag == 3)
//         {
//             rgb_matrix_set_color_all(0,0,0);
//             rgblight_set_layer_state(1,true);
//         }


//         if(sleep_switch_driverflag == 0)
//         {
//             rgblight_set_layer_state(1,false);
//             setPinOutput(ARGB_LEFT_EN);
//             writePinHigh(ARGB_LEFT_EN);
     
         
//         }
//    }



   //恢复原来的灯效
      
//    if( rgblight_get_mode() != (RGBLIGHT_MODE_SNAKE))
//    {
//       old_effect  = rgblight_get_mode();
//    }
       
//    if(rgblight_get_mode() == (RGBLIGHT_MODE_SNAKE) && ( == 1) && (Plug_In_Flag == 0))
//    {
//         Default_Effect_Flag = 0;
//         rgblight_mode_noeeprom(old_effect);//

//    }




// 
  





    //uprintf("last_wireless_mode:%ld\r\n",last_wireless_mode);
    //uprintf("layer_state :%0x\r\n",layer_state );
    //uprintf("default_layer_state :%0x\r\n",default_layer_state);
    //uprintf("keymap_config.nkro :%d\r\n",keymap_config.nkro); 
    //CHRG_EN_PIN引脚拉低时，且bat_test_flag值为0是会获取adc值（只会执行一次）
     //uprintf("FN_Count:%d\r\n",FN_Count);
          
  

   // uprintf("last_wireless_mode:%d\r\n", last_wireless_mode);
   //timer_elapsed32(Power_Full_timer) 
  
    current_layer = get_highest_layer(layer_state | default_layer);
   // uprintf("temp:%d\r\n",temp);
 //   uprintf("FN_Count:%d\r\n",FN_Count);
    if((bat_test_flag == false))
    {
        adc_value = get_adc_value();    //获取adc值
        adc_vref =  get_adc_vref();
        battery_value = (adc_value*1764/adc_vref);
        battery_test_time = timer_read32();
        temp = batt_level();
        last_temp = temp;
        bat_test_flag = true;              //把bat_test_flag置1，确保只进一次
    }

    adc_test();         //adc检测
    ble_send_batt();    //蓝牙发送百分比电量


//防止按键按下时间过长
    if(timer_elapsed32(First_timer) > 400)
    {
       FN_Count = 0;
    }

    if(timer_elapsed32(Media_timer) > 100 && (Media_Flag == 1) && (FN_Count == 0))
    {
        Media_Flag = 2;

        current_layer = get_highest_layer(layer_state | default_layer);
        if(current_layer == 0) 
        {
            set_single_persistent_default_layer(4);
        }
        else if(current_layer == 2)
        {
            set_single_persistent_default_layer(6);
        }
        else if(current_layer == 4)
        {
            set_single_persistent_default_layer(0);
        }
        else if(current_layer == 6)
        {
            set_single_persistent_default_layer(2);
        }
    }

   //检测电池发送，空闲才发送
    if((battery_Flag == 1) && (!keyboard_Idle) )
    {
        battery_Flag = 0;
        sc_ble_battary(temp); 
    }

//question
      get_mode();
//    上电进来一次默认为usb
    if (kb_mode != new_kb_mode)  //ONLY DO IT ONCE WHEN MODE SWITCHED
    {
         new_kb_mode = kb_mode;  
         BT_24G_Shine = 0;   //，模式切换进来一次，BT_24G_Shine = 0；
        if(kb_mode==KB_MODE_BLE)
        {
            last_wireless_mode &= 3;
            WIRELESS_START(last_wireless_mode);
            flag32.link_break_flag =0;
            rgb_wireless_timer = timer_read32();
            first_sleep_timer = timer_read32();
            packet_send = 0;
        }


        else if(kb_mode == KB_MODE_24G)
        {
            WIRELESS_START(4);
            flag32.link_break_flag =0;
            rgb_wireless_timer = timer_read32();
            first_sleep_timer = timer_read32();
            packet_send = 0;
        }


        else if(kb_mode == KB_MODE_USB)
        {
            WIRELESS_STOP();
            pair_succeed_timer = timer_read32();
            blink_counter = 0;
        }


        blink_ble_24g_timer = 0; //蓝牙回连超时     配对也是这个  
        pair_succeed_timer = timer_read32();
        //重新计时连接时间  
        blink_ble_24g_timer = timer_read32(); //重新计时回连时间
        pair_timeout = 20000;
    //    BT_Switch_Flag = 1;
        ble24G_period = 5;
        ble24G_counter  = 0;
        flag32.mode_one = 1;
    }






    if(1 == flag32.press_24G_flag)
    {
        if(timer_elapsed32(long_ble_24g_timer) > BLE_24G_TIMEOUT)
        {

            flag32.link_break_flag =0;
            flag32.press_24G_flag = 0;
            WIRELESS_PAIR(4);
            blink_ble_24g_timer = timer_read32();            
            ble24G_period = 2;
            ble24G_counter  = 0;
            pair_timeout = 60000;
          //  BT_Switch_Flag = 0;
            flag32.mode_one = 1;
        }
    }
     

    if(1 == flag32.press_ble_flag)
    {
        if(timer_elapsed32(long_ble_24g_timer) > BLE_24G_TIMEOUT)
        {
            flag32.link_break_flag =0;

            // ble_led_state = 0;
            flag32.press_ble_flag = 0;
            WIRELESS_PAIR(last_wireless_mode);
            blink_ble_24g_timer = timer_read32();
            ble24G_period = 2;
            ble24G_counter  = 0;
            pair_timeout = 60000;
          //  BT_Switch_Flag = 0;
            flag32.mode_one = 1;
        }
    }
    //以上


             
            
        //长按3s复位键处理
        if(1 == flag32.reset_flag)
        {
            if(timer_elapsed32(long_press_reset_timer) > RESET_TIMEOUT)
            {
    
                flag32.reset_flag = 0;
                blink_num = 3;
                Valorspd_counter = 0;
                reset_flag = 1;
            }
        }


        if(!wireless_connected)
        {
    
            if( (flag32.link_break_flag == 1))
            {
                flag32.link_break_flag = 0;
                ble24G_period = 5;
                ble24G_counter = 0;
                flag32.mode_one = 1;
                connect_buff = 0;
                pair_timeout = 20000;  
               // BT_Switch_Flag = 1;
                blink_ble_24g_timer = timer_read32(); //重新计时回连时间  
            }
        }
        if(wireless_connected)
        {
            if(connect_buff == 0)
            {
                connect_buff = 1;
                rgb_wireless_timer = timer_read32();
                first_sleep_timer = timer_read32();
                packet_send = 0;
            }
        }
       sleep_mode();
 

    return LOOP_10HZ_PERIOD;
}




uint8_t test01 = 1;
uint8_t unchrg_falg = 1;
uint8_t chrg_falg = 1;
uint8_t chrgfull_falg = 1;
uint8_t chrg_battest = 1;
uint8_t lowpower_effect;

bool rgb_matrix_indicators_advanced_kb(uint8_t led_min, uint8_t led_max) 
{



   if(sleep_switch_driverflag) 
   {
        rgb_matrix_set_color_all(0,0,0);
        rgblight_set_layer_state(1,true);
        sleep_switch_driverflag = sleep_switch_driverflag -1;
  

        if(sleep_switch_driverflag == 50)
        { 
            //侧灯所有开关全关掉
            rgblight_set_layer_state(0,false);
            for(uint8_t i = 2;i < 12; i++)
            {
              rgblight_set_layer_state(i,false);
            }  
            setPinOutput(ARGB_LEFT_EN);
            writePinHigh(ARGB_LEFT_EN);
        }




        if(sleep_switch_driverflag == 0)
        {
            rgblight_set_layer_state(1,false);

        }
   }


        // if(sleep_switch_driverflag)
        // {
        //   rgb_matrix_set_color_all(0,0,0);
        // }


    if(sleep_switch_driverflag == 0)
    {
            if(shut_up_Flag || lock_keyboard )
            {
                rgb_matrix_set_color_all(0,0,0);
                rgblight_set_layer_state(1,true);
            }

            else{
                rgblight_set_layer_state(1,false);
            }


            if(((!wireless_connected) && ( (kb_mode == KB_MODE_BLE)   || ( kb_mode == KB_MODE_24G)))   || (lock_keyboard))
            {
                rgb_matrix_set_color_all(0,0,0);
            } 

            if(lock_keyboard)
            {
                rgb_matrix_set_color(34,255,255,255);
            }
            
            #ifdef Effect_MK637



   // use MATRIX LEDS  as RGBLIGHT LEDS ..JackyJia
        extern ws2812_led_t ws2812_leds[WS2812_LED_COUNT];
        for (uint8_t i=0;i<2;i++)  //3,4
        { 
            ws2812_set_color(3+i, ws2812_leds[i].r,ws2812_leds[i].g,ws2812_leds[i].b);
        }
//        ws2812_flush();

#endif

            // //旋钮的优先级低于模式
            // if(!BT_24G_Shine)
            // {
            //     if(encode_toggle_flag)
            //     {
            //       if(timer_elapsed32(encoder_longpresstimer) < 1200)
            //       { 
            //                 rgblight_set_layer_state(12,true);
            //       }
            //       else
            //       {
            //                  encode_toggle_flag = 0;
            //                 rgblight_set_layer_state(12,false);
            //       }
            //     }
            // }
            // else //防止闪灯期间切回模式等效
            // {
            //              encode_toggle_flag = 0;
            //              rgblight_set_layer_state(12,false);
            // }
    





                
            //亮度和速度灯效
        // if(!BT_24G_Shine)
            {
                if (blink_num != 0) {  
                    BT_24G_Shine = 0;
                    rgblight_set_layer_state(6,false);
                    rgblight_set_layer_state(3,false);
                    rgblight_set_layer_state(4,false);
                    rgblight_set_layer_state(5,false);
                    rgblight_set_layer_state(7,false);
                    rgblight_set_layer_state(8,false);
                    rgblight_set_layer_state(9,false);
                    rgblight_set_layer_state(10,false);
                    rgblight_set_layer_state(11,false);
                    if (Valorspd_counter < Valorspd_period)
                    {   

                        rgblight_set_layer_state(0,true); //亮
                        rgblight_set_layer_state(1,false); 
                    }
                    else
                    {
                        rgblight_set_layer_state(0,false); //亮
                        rgblight_set_layer_state(1,true); 
                    }
                } 
                else { 
                    if(reset_flag)
                    {
                        rgblight_set_layer_state(0,false);
                        rgblight_set_layer_state(1,false);
                        eeconfig_init();
                        mcu_reset();   
                    }
                }
            }


           if( ((host_keyboard_leds() & 0x02) == 0x02) && (wireless_connected || ((kb_mode == KB_MODE_USB) && (get_plug_mode() == true))  ))
            {
                rgb_matrix_set_color(25,255,255,255);
                
            }


            //电量指示
            if(!get_plug_mode() &&  Light_Count && ((kb_mode == KB_MODE_BLE) || (kb_mode == KB_MODE_24G))  )   //
            {      
                rgb_matrix_set_color_all(0,0,0); 
                for(uint8_t i = 0;i < Light_Count; i++ )
                {
                    rgb_matrix_set_color(55+i,255,255,255);
                }

                if(Light_Count > 0 && (Light_Count <= 3))
                {
                    for(uint8_t i = 0;i < Light_Count; i++ )  //紅      紅綠可以混成黃色
                    {
                        rgb_matrix_set_color(55+i,255,0,0);
                    }
                }

                if(Light_Count > 3 && (Light_Count <= 7))
                {
                    for(uint8_t i = 0;i < Light_Count; i++ )  //黃    紅綠可以混成黃色
                    {
                        rgb_matrix_set_color(55+i,255,255,0);
                    }
                }

                if(Light_Count > 7 && (Light_Count <= 10))
                {
                    for(uint8_t i = 0;i < Light_Count; i++ )  //黃    紅綠可以混成黃色
                    {
                        rgb_matrix_set_color(55+i,0,255,0);
                    }
                }
                // if(temp <30)
                // {
                //     rgblight_set_layer_state(5,true);
                // }
                // else if(temp >=30 && (temp <70))
                // {
                //     rgblight_set_layer_state(4,true);
                // }
                // else{
                //     rgblight_set_layer_state(9,true);
                // }
            }

            if(0 == lock_win_flag && (wireless_connected || (kb_mode == KB_MODE_USB)))
            {   
                    if(default_layer == 1 ||(default_layer == 16))
                rgb_matrix_set_color(22,255,255,255);
            }



        // //充电灯效
        //     if(get_plug_mode() && (temp < 100) && (bat_test_flag == true) && Test_Battery_value)
        //     { 
        //        rgb_matrix_set_color(33,255,0,0);
        //     }
        //低电灯效

        //  充电与充满灯效    //防止上电时看起来灯效会覆盖太快，限制下时间

        if(!BT_24G_Shine && (blink_num == 0))      
        {
                if(get_plug_mode()  && (!lowpower_flag))   
                {
                    //增加一个充电标志，防止via切换充电灯效
                
                //低电处理
                    if(lowpower_effect == 1)
                    {
                        lowpower_effect = 2;
                        rgblight_set_layer_state(11,false);
                        rgblight_set_layer_state(10,false);
                    }


                    if(temp < 100)   // chrg in  
                    {
                       chag_via_flag = 1;
                       //这里开始计时
                       if(!chag_first_flag)
                       {
                          chag_first_flag = 1;
                          keyborad_Chrgtimer = timer_read32(); 
                       }
                      if(timer_elapsed32(keyborad_Chrgtimer) < 5000)
                      {
                        rgblight_set_layer_state(9,true);
                      }
                      else{
                        rgblight_set_layer_state(9,false);
                      }      
                    }
                    else{
                        chag_first_flag = 0;

                        chag_via_flag = 0; 
                        rgblight_set_layer_state(9,false);
                    }


                    // if(temp < 100)  //充电与充满互相转换
                    // {
                    //     // if(!Plug_In_Flag)
                    //     // {
                    //         // Plug_In_Flag = 1;
                    //         Default_Effect_Flag = 1;
                    //         rgblight_mode_noeeprom(RGBLIGHT_MODE_SNAKE); //充电灯效时不可以刷其他灯效
                    //     // }
                    //     //  rgblight_set_layer_state(9,false);
                    // }
                    // else{  //充满
                    
                    //      //开始计时 
                    //      if(!Power_Full_Flag)   
                    //      {
                    //         Power_Full_Flag = 1;
                    //         Power_Full_timer = timer_read32();
                    //         rgblight_set_layer_state(9,true);
                    //      } 

                    //      if(timer_elapsed32(Power_Full_timer) >= 10000   && (Power_Full_Flag))  // 超时10s,这里有bug，第二次进去会有问题
                    //      {
                    //        //  Power_Full_Flag = 0;
                    //          rgblight_set_layer_state(9,false);
                    //      }
                    // }
                }
                else
                {        

                    chag_first_flag = 0;  


                    chag_via_flag = 0;
                    Plug_In_Flag = 0;
                    rgblight_set_layer_state(9,false);
                    // Power_Full_Flag = 0;
                    //低电报警
                    lowpower_effect = 1;
                    
        
                    if(!lowpower_flag && ((temp <=10)))
                    {
                        if(power_counter < power_period)
                        {
                            rgblight_set_layer_state(10,true);  //亮红
                            rgblight_set_layer_state(11,false);
                        }
                        else
                        {
                            rgblight_set_layer_state(10,false);//亮黑
                            rgblight_set_layer_state(11,true);
                        }
                    }
                }

        }
        else
        {
            //模式灯效中关闭低电
                    chag_via_flag = 0;
                    rgblight_set_layer_state(10,false);  //亮红
                    rgblight_set_layer_state(11,false);
                    rgblight_set_layer_state(9,false);
        }
        if(1 == flag32.mode_one)
        {
            if (blink_num == 0) {
            three_mode();
            }
        }
    }



    return false;
}





bool process_record_kb(uint16_t keycode, keyrecord_t* record) 
{

    if(record->event.pressed)
    {
        rgb_wireless_timer  = timer_read32();
        blink_ble_24g_timer = timer_read32();
        first_sleep_timer   = timer_read32(); 
  //      uprintf("keycode :%d\r\n",keycode);
       if (!wireless_connected) 
       {
            for (int i=0;i<5;i++)
            {
                uart_write(0x00);
            }
       }
        packet_send = 0;
    }


    if(lock_keyboard)//锁键盘
    {  
       // if((keycode != KC_LOCK  ||(keycode != MO(1))   ||(keycode != MO(3)))) //防止上锁后解锁不了
       if(keycode != MO(1) && (keycode != MO(3))  && (keycode != MO(5))   && (keycode != MO(7)) && (keycode != KC_LOCK))
        {
             if(record->event.pressed)
             {
                // uprintf("enter here\r\n");
                // uprintf("enter here\r\n");
                // uprintf("enter here\r\n");
                // uprintf("enter here\r\n");
                // uprintf("enter here\r\n");
                return false;
             }  
  
        } 
    }
    


    if(keycode == KC_LOCK)
    {
       if(record->event.pressed)  
        {
            //松开按键，清掉fn
            lock_keyboard = ~lock_keyboard;
        }
        return false;
    } 

//  


   


      



//增加双击fn切层的功能
    if(keycode ==  MO(5) || (keycode ==  MO(7)) ||  keycode ==  MO(1) || (keycode ==  MO(3)))
    {

        if(record->event.pressed)  
        {

             First_timer = timer_read32();
             FN_Count++;
            if(timer_elapsed32(Doubele_Fn_timer) < 200 && (FN_Count == 1))//第三次快速按下时清0
            {
                FN_Count = 0;
                Media_Flag = 0;
                return true; //第三次直接推出去

            }
      
            //第一次松开到第二次按下的时间间隔小于
            if(timer_elapsed32(Doubele_Fn_timer) < 200 && (FN_Count == 2))
            {
                //开始计时，超过200ms
                //切层 
               FN_Count = 0;
               Media_Flag = 1; 
               Media_timer = timer_read32();             
            }
            if(FN_Count == 2)
            {
                FN_Count = 0;
            }
        }
        else{
            //  if(timer_elapsed32(First_timer) > 400)
            //  {
            //     FN_Count = 0;
            //  }
            //松开开始计时           
            Doubele_Fn_timer = timer_read32();
        }
        return true;
    } 


// //增加长按切换旋钮的功能 ENC_TG
//     if (keycode == ENC_TG) {
//         if (record->event.pressed) {
//             encode_toggle_timer = timer_read32();   
//             encode_Press_flag = 1;    
//         }
//         else{
//                if(timer_elapsed32(encode_toggle_timer) <= 2000)
//                {//防止长按松开会切换功能
//                     if (!encode_toggle) 
//                     {
//                         tap_code(KC_MUTE);//按下时的功能
//                     }
//                     else{
//                         rgb_matrix_step();
//                     }
//                     encode_toggle_timer = timer_read32();
//                }
//                encode_Press_flag = 0;
//         }
//        return false;
//     }

// 0    1    2     3   4    5    6     7
// win      mac       win        mac
// 1    2    4     8   16  32    64   128    default


   //windows和IOS切换处理
    if(keycode== KC_WINMODE)
    {
        if(record->event.pressed)  
        {
            if(default_layer == 4)
            {
              set_single_persistent_default_layer(0); //Sets the default layer and writes it to persistent memory (EEPROM).
            }
            else if(default_layer == 64)
            {
                set_single_persistent_default_layer(4);
            }

        }
        return false;
    } 





// 0    1    2     3   4    5    6     7
// win      mac       win        mac
// 1    2    4     8   16  32    64   128    default



    if(keycode==KC_MACMODE)
    {
        if (record->event.pressed)  
        {
            //fn双击以后切到第6层
              if(default_layer == 1)
              {
                 set_single_persistent_default_layer(2); //Sets the default layer and writes it to persistent memory (EEPROM)
              }
              else if(default_layer == 16)
              {
                set_single_persistent_default_layer(6);
              }
        }
        return false;
    } 


    if ((keycode == KC_SHUTRGB) && record->event.pressed )
    {
        shut_up_Flag = ~ shut_up_Flag;
    }


       //电量检测
    if(keycode == KC_BAT)
    {
        if (record->event.pressed) 
        { 
            Test_Battery_value = 1;
            Light_Count = temp/10 +1;
            if(Light_Count > 10)
            Light_Count = 10;        
        }
        else
        {
            Test_Battery_value = 0;
            Light_Count = 0;
            // rgblight_set_layer_state(5,false);
            // rgblight_set_layer_state(4,false);
            // rgblight_set_layer_state(9,false);
        }
    }


     

     

    //按键切换模式   
     if( (keycode>=KC_BLE1) && (keycode<=KC_BLE3) && (kb_mode==KB_MODE_BLE))
    {
        flag32.link_break_flag =0;
        flag32.ble_report_flag = 0;
        if(last_wireless_mode != keycode-KC_USB)  
        {
            if (record->event.pressed)
            {         
                ble_mode = keycode-KC_USB;
                WIRELESS_START(ble_mode);
                blink_ble_24g_timer = timer_read32();
                long_ble_24g_timer=timer_read32();
                flag32.mode_one = 1;
                ble24G_period = 5;
            //   uprintf("problem2\r\n");
                ble24G_counter  = 0;
                pair_timeout = 20000; 
              //  BT_Switch_Flag = 1;
                //解决蓝牙互切之间也可以不松手直接配对    
                long_ble_24g_timer=timer_read32();
                flag32.press_ble_flag = 1; 
                            
            }
        }
        else
        {               
            if (record->event.pressed)
            {
                long_ble_24g_timer=timer_read32();
                flag32.press_ble_flag = 1;
            }
            else
            {
                if(timer_elapsed32(long_ble_24g_timer) <= BLE_24G_TIMEOUT)   
                {
                    flag32.press_ble_flag = 0;              
                }
            }
            
        }
        return false;     
    }

    if (keycode == KC_24G && (kb_mode==KB_MODE_24G))
    {  
        {
            if(record->event.pressed)
            {
                long_ble_24g_timer = timer_read32();
                flag32.press_24G_flag = 1;

            }
            else
            {
                if (timer_elapsed32(long_ble_24g_timer) <= BLE_24G_TIMEOUT)    //长按广播配对
                {
                    flag32.press_24G_flag = 0;        
                }
            }
        }

        return false;
    }

    if (keycode == KC_Mctl) {

        // uprintf("mctl\r\n");
        if (record->event.pressed)
            register_code16(KC_MCTL);
        else
            unregister_code16(KC_MCTL);
    }
    if (keycode == KC_Lpad) {
        
       //  uprintf("lpad\r\n");
        if (record->event.pressed)
            register_code16(KC_LPAD);
        else
            unregister_code16(KC_LPAD);
    }


//question
    
    if(((keycode==KC_WIN)) && record->event.pressed)
    {
    
        lock_win_flag = !lock_win_flag;
      //  dynamic_keymap_set_keycode(1, 2, 0,lock_win_flag);
        variable_data.eeconfig_lock_win_flag = lock_win_flag;
        eeconfig_update_kb_datablock(&variable_data, 0, EECONFIG_KB_DATA_SIZE);

    }

    
 

    if(record->event.pressed && ((keycode == KC_LGUI) || (keycode == KC_RGUI)))
    {
        if(default_layer == 1 || (default_layer == 16))  //第0层为1，第5层为16
        { 
            if(lock_win_flag== 1)
            return true;
            else
            return false;
            
        }
    }


    switch(keycode)
    {
        case KC_RESET:
            if (record->event.pressed)  //恢复出厂设置 
            {
                flag32.reset_flag = 1;
                long_press_reset_timer = timer_read32();  //按下按键时就给个标志位，开始计时
            }
            else                        //松手进入一次else
            {
                flag32.reset_flag = 0;
            }
        break;

//#if (defined(RGBLIGHT_ENABLE) && !defined(RGBLIGHT_DISABLE_KEYCODES)) || (defined(RGB_MATRIX_ENABLE) && !defined(RGB_MATRIX_DISABLE_KEYCODES))
//            uint8_t shifted = get_mods() & MOD_MASK_SHIFT;
//#endif
#ifdef Right_Switch_MK637
        case KC_MODEPLUS:
            if(!chag_via_flag)
                rgblight_step_noeeprom();
//            uprintf("enter here\r\n");
            break;
        case KC_HUEPLS:
            if(!chag_via_flag)
               rgblight_increase_hue_noeeprom(); 
            break;

        case KC_VAL_UP:
            if(!chag_via_flag) {
                rgblight_increase_val_noeeprom();
            }
            break;

        case KC_VAL_DN:
            if(!chag_via_flag)
                rgblight_decrease_val_noeeprom();
             break;
            //  case KC_SPD_UP:
            //  handleKeycodeRGB(shifted, rgblight_increase_speed, rgblight_decrease_speed);
            // return false;

            // case KC_SPD_DN:
            //   handleKeycodeRGB(shifted, rgblight_decrease_speed, rgblight_increase_speed);
#endif
        default:
            break;

    }   
    return process_record_user(keycode, record);   
}






bool encoder_update_kb(uint8_t index, bool clockwise) {

        // uprintf("problem2\r\n");
        // uprintf("problem2\r\n");
        // uprintf("problem2\r\n");
        // uprintf("problem2\r\n");
        // uprintf("problem2\r\n");
        // uprintf("problem2\r\n");
        //增加限制，锁键盘也不可以旋转切换
        if(!lock_keyboard)
        {
            // if (encode_toggle == 1) { /* First encoder */
            //     if (clockwise) {
            //         rgb_matrix_decrease_val();
            //     } else {
            //         rgb_matrix_increase_val();
            //     }
            // } else if (encode_toggle == 0) { /* Second encoder */
            //     if (clockwise) {
            //         tap_code(KC_VOLD);
            //     } else {
            //         tap_code(KC_VOLU);
            //     }
            // }
        }

    return false;
}




 void three_mode(void)
{
    switch(kb_mode)
    {
        case KB_MODE_USB:
                        rgblight_set_layer_state(6,false);
                        rgblight_set_layer_state(3,false);
                        rgblight_set_layer_state(4,false);
                        rgblight_set_layer_state(5,false);
                        rgblight_set_layer_state(7,false);
                        rgblight_set_layer_state(8,false);
                    
        break;

        case KB_MODE_BLE:

                rgblight_set_layer_state(7,false);
                rgblight_set_layer_state(8,false);
                if(!wireless_connected)
                {             
                    if(timer_elapsed32(blink_ble_24g_timer) <= pair_timeout)
                    {
                        switch (last_wireless_mode) 
                        {
                            case 1:
                                 rgblight_set_layer_state(4,false);
                                 rgblight_set_layer_state(5,false);
                                if(ble24G_counter < ble24G_period)
                                {
                                    // if(BT_Switch_Flag)
                                    // rgb_matrix_set_color(52,0,255,0); //蓝牙回连绿色，配对蓝色
                                    // else
                                     rgb_matrix_set_color(52,0,0,255);
                               
                                    rgblight_set_layer_state(3,true);
                                    rgblight_set_layer_state(6,false);
                                }
                                else
                                {     
                                     rgblight_set_layer_state(6,true);
                                     rgblight_set_layer_state(3,false);
                                    rgb_matrix_set_color(52,0,0,0);
                                }
                            break;

                            case 2: 
                                 rgblight_set_layer_state(3,false);
                                 rgblight_set_layer_state(5,false); 
                                if(ble24G_counter < ble24G_period)
                                {
                                    rgblight_set_layer_state(6,false);
                                    rgblight_set_layer_state(4,true);;
                                
                                }
                                else
                                {
                                     rgblight_set_layer_state(4,false);
                                     rgblight_set_layer_state(6,true);
                                  
                                }
                            break;

                            case 3:
                                 rgblight_set_layer_state(4,false);
                                 rgblight_set_layer_state(3,false);  
                                if(ble24G_counter < ble24G_period)
                                {
                                    rgblight_set_layer_state(6,false);
                                    rgblight_set_layer_state(5,true);;
                                }
                                else
                                {
                                     rgblight_set_layer_state(5,false);
                                     rgblight_set_layer_state(6,true);
                                  
                                }     
                            break;
                            default:
                            break;
                        }
                    }
                    else{
                        rgblight_set_layer_state(6,false);
                        rgblight_set_layer_state(3,false);
                        rgblight_set_layer_state(4,false);
                        rgblight_set_layer_state(5,false);
                        POWER_EnterSleep();  
                         BT_24G_Shine = 0;  
                    }
                    pair_succeed_timer = timer_read32();
                    if (blink_num == 0) { 
                      BT_24G_Shine = 1;
                    }
                }
                else    
                {
                     rgblight_set_layer_state(6,false);
                    flag32.link_break_flag = 1;
                    if(timer_elapsed32(pair_succeed_timer) <= PAIR_SUCCEED_TIME)
                    {
                          rgblight_set_layer_state(6,false);
                        switch (last_wireless_mode) 
                        {
                            case 1:
                                     rgblight_set_layer_state(3,true);
                                     
                            break;

                            case 2:   

                                      rgblight_set_layer_state(4,true);
                            break;

                            case 3:   

                                      rgblight_set_layer_state(5,true);
                            break;
                            default:
                                break;
                        }
                        BT_24G_Shine = 1; //蓝牙没有连上
                    }
                    else
                    {
                         BT_24G_Shine = 0; 
                        rgblight_set_layer_state(3,false);
                        rgblight_set_layer_state(4,false);
                        rgblight_set_layer_state(5,false);
                        rgblight_set_layer_state(6,false);
                        flag32.mode_one = 0;                                          
                    }  
                }
        break;

        case KB_MODE_24G:

                    rgblight_set_layer_state(6,false);
                    rgblight_set_layer_state(3,false);
                    rgblight_set_layer_state(4,false);
                    rgblight_set_layer_state(5,false);
                if(!wireless_connected)
                {
                    if(timer_elapsed32(blink_ble_24g_timer) <= pair_timeout)
                    {
                        if(ble24G_counter < ble24G_period)
                        {
                                rgblight_set_layer_state(8,false);
                                rgblight_set_layer_state(7,true);
                        }
                        else 
                        {
                               rgblight_set_layer_state(7,false);
                               rgblight_set_layer_state(8,true);
                        } 
                    }
                    else{
                                rgblight_set_layer_state(7,false);
                                rgblight_set_layer_state(8,false);
                                POWER_EnterSleep();
                                BT_24G_Shine = 0;  
                    }
                    pair_succeed_timer = timer_read32();
                    if (blink_num == 0) { 
                      BT_24G_Shine = 1; 
                    }
                }
                else
                {
                    flag32.link_break_flag = 1;
                    if(timer_elapsed32(pair_succeed_timer) <= PAIR_SUCCEED_TIME)
                    {
                          rgblight_set_layer_state(8,false);
                          rgblight_set_layer_state(7,true);
                    }
                    else
                    {
                          rgblight_set_layer_state(7,false);
                          rgblight_set_layer_state(8,false);
                           BT_24G_Shine = 0;                                 
                    } 
                    
                   if(wakeup_pack24G)
                   {
                        wakeup_pack24G = 0;
                        tap_code(KC_F24);
                   }
                }
        break;
        default:
        break;
    }

}





void gpio_disable_init(void)
{
    setPinOutput(ARGB_LEFT_EN);
    writePinLow(ARGB_LEFT_EN);

      //  setPinInput(ARGB_LEFT_EN);
    palSetLineMode(OSC_IN,PAL_MODE_INPUT_ANALOG);  //0.23ma
    palSetLineMode(OSC_OUT,PAL_MODE_INPUT_ANALOG);
}
//question
void gpio_init(void)
{
    //插入检测引脚
   setPinInput(PLUG_IN);
    //晶振引脚初始
   setPinInput(OSC_IN);
   setPinOutput(OSC_OUT);
}





void usb_suspend_power_down(void)
{
    sleep_switch_driverflag = 200;
    release_count = 4;
    cancel_deferred_exec(loop10hz_token);
    gpio_disable_init();
    setPinInput(BLE);
    setPinInput(TwoMode);
    palEnableLineEvent(BLE, PAL_EVENT_MODE_BOTH_EDGES);
    palEnableLineEvent(TwoMode, PAL_EVENT_MODE_BOTH_EDGES);

    setPinInput(encoder_left);
    setPinInput(encoder_right);
    palEnableLineEvent(encoder_left, PAL_EVENT_MODE_RISING_EDGE);
    palEnableLineEvent(encoder_right, PAL_EVENT_MODE_RISING_EDGE);
    #if (DIODE_DIRECTION == ROW2COL)

    pin_t col_pins[] = MATRIX_COL_PINS;

    for(uint8_t x=0; x<MATRIX_COLS; x++)
    {
        pin_t pin;
        pin = col_pins[x];
        if (pin != NO_PIN)
        {
            setPinOutput(pin);
            writePinLow(pin);

        }
    }


    const long unsigned int row_pins[] = MATRIX_ROW_PINS;
    for(uint8_t x=0; x<MATRIX_ROWS; x++)
    {
        pin_t pin;
        pin = row_pins[x];
        if (pin != NO_PIN)
        {
            setPinInputHigh(pin);
            palEnableLineEvent(pin, PAL_EVENT_MODE_BOTH_EDGES);
        }
    }
#else

    const long unsigned int row_pins[] = MATRIX_ROW_PINS;
    for(uint8_t x=0; x<MATRIX_ROWS; x++)
    {
        pin_t pin;
        pin = row_pins[x];
        if (pin != NO_PIN)
        {
            setPinOutput(pin);
            writePinLow(pin);
        }
    }

    const long unsigned int col_pins[] = MATRIX_COL_PINS;
    for(uint8_t x=0; x<MATRIX_COLS; x++)
    //for(uint8_t x=0; x<9; x++)
    {
        pin_t pin; 
        pin = col_pins[x];
        setPinInputHigh(pin);
        palEnableLineEvent(pin, PAL_EVENT_MODE_BOTH_EDGES);
    }
#endif
   ADC1->CR2 &= ~ADC_CR2_ADON;
    _pal_lld_enablepadevent(0,18,PAL_EVENT_MODE_BOTH_EDGES);

    PWR->CR |= PWR_CR_CWUF | PWR_CR_CSBF;
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    __WFI();
     SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    #ifdef  mk637_special_isr
    special_key_debounce();
    #endif
//休眠防抖
//休眠防抖
    swif_debounce();
    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    EXTI->IMR &= ~(1<<18);
    EXTI->EMR &= ~(1<<18);
    EXTI->RTSR &= ~(1<<18);
    EXTI->FTSR &= ~(1<<18);
    EXTI->PR = (1<<18);
    setPinInput(BLE);
    setPinInput(TwoMode);
    stm32_clock_init();
    clear_keyboard();
    send_keyboard_report();
    usbWakeupHost(&USB_DRIVER);  //解决休眠无法唤醒的问题
    restart_usb_driver(&USB_DRIVER);
    init_usb_driver(&USB_DRIVER); //Should not enter SLEEP when USB mode
    matrix_init();
    ws2812_init();
    adc_init();
    gpio_init();
    get_mode();
    if(kb_mode == KB_MODE_BLE)
    {
        WIRELESS_START(last_wireless_mode);
    }
    if(kb_mode == KB_MODE_24G)
    {
        WIRELESS_START(4);
    }

    

    usb_suspend_flag = 2;
    power_counter = 0;   
    loop10hz_token = defer_exec(LOOP_10HZ_PERIOD, loop_10Hz, NULL);
}



uint8_t Sleep_First_PressFlag = 0;
void POWER_EnterSleep_First(void) {
    Sleep_First_PressFlag = 1;
    sleep_switch_driverflag = 200;
    cancel_deferred_exec(loop10hz_token);
    usb_disconnect();    //Don't enter SLEEP when USB mode
    gpio_disable_init();
    palSetLineMode(RENUM,PAL_MODE_INPUT_ANALOG);
    if(get_plug_mode() == false)
    {
        palSetLineMode(A11,PAL_MODE_INPUT_ANALOG);
        palSetLineMode(A12,PAL_MODE_INPUT_ANALOG);
    }
    setPinInput(BLE);
    setPinInput(TwoMode);
    palEnableLineEvent(BLE, PAL_EVENT_MODE_BOTH_EDGES);
    palEnableLineEvent(TwoMode, PAL_EVENT_MODE_BOTH_EDGES);


    setPinInput(encoder_left);
    setPinInput(encoder_right);
    palEnableLineEvent(encoder_left, PAL_EVENT_MODE_RISING_EDGE);
    palEnableLineEvent(encoder_right, PAL_EVENT_MODE_RISING_EDGE);
#if (DIODE_DIRECTION == ROW2COL)

    pin_t col_pins[] = MATRIX_COL_PINS;

    for(uint8_t x=0; x<MATRIX_COLS; x++)
    {
        pin_t pin;
        pin = col_pins[x];
        if (pin != NO_PIN) 
        {
            setPinOutput(pin);
            writePinLow(pin);
        }
    }

    const long unsigned int row_pins[] = MATRIX_ROW_PINS;
   for(uint8_t x=0; x<MATRIX_ROWS; x++)
    { 
       pin_t pin;
       pin = row_pins[x];
        if (pin != NO_PIN) 
        {
            setPinInputHigh(pin);
            palEnableLineEvent(pin, PAL_EVENT_MODE_FALLING_EDGE);
        }
    }
   setPinInput(PLUG_IN);
   palEnableLineEvent(PLUG_IN, PAL_EVENT_MODE_BOTH_EDGES);
#else

    const long unsigned int row_pins[] = MATRIX_ROW_PINS;
    for(uint8_t x=0; x<MATRIX_ROWS; x++)
    { 
        pin_t pin;
        pin = row_pins[x];
        if (pin != NO_PIN) 
        {
            setPinOutput(pin);
            writePinLow(pin);
        }
    }

    const long unsigned int col_pins[] = MATRIX_COL_PINS;
    for(uint8_t x=0; x<MATRIX_COLS; x++)
    //for(uint8_t x=0; x<9; x++)
    {
        pin_t pin;
        pin = col_pins[x];
        setPinInputHigh(pin);
        palEnableLineEvent(pin, PAL_EVENT_MODE_BOTH_EDGES);
    }
#endif

 
    ADC1->CR2 &= ~ADC_CR2_ADON;   //0.9ma   
    //writePinLow(DRIVER_1_EN); 
    /* going to anabiosis*/
    //chSysLock();
    PWR->CR |= (1<<0)|(1<< 10) |(1<< 11)|(3<<18);
    /* Clear Wake-up flag */
    PWR->CR |= PWR_CR_CWUF | PWR_CR_CSBF;

    /* Set SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    //}
    /* Request Wait For Interrupt */
    __WFI();
     SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
     #ifdef  mk637_special_isr
    special_key_debounce();
#endif
//休眠防抖
//休眠防抖
    swif_debounce();
    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    setPinInput(BLE);
    setPinInput(TwoMode);
    setPinOutput(RENUM);
    writePinHigh(RENUM);
    gpio_init();
    matrix_init();
    matrix_scan();
    stm32_clock_init();
    ws2812_init();
    adc_init(); 
    get_mode();
    rgb_wireless_timer = timer_read32();
     first_sleep_timer = timer_read32();
     packet_send = 0;
    //配对超时和回连超时计时清0

    blink_ble_24g_timer = 0; //蓝牙回连超时     配对也是这个  
    pair_succeed_timer = timer_read32();//重新计时连接时间  
    blink_ble_24g_timer = timer_read32(); //重新计时回连时间
    flag32.mode_one = 1;
    blink_counter  = 0;
    ble24G_counter = 0;
    power_counter = 0;   
    loop10hz_token = defer_exec(LOOP_10HZ_PERIOD, loop_10Hz, NULL);
  }





void POWER_EnterSleep(void) {
    sleep_switch_driverflag = 200;
    cancel_deferred_exec(loop10hz_token);
    WIRELESS_STOPOWER();
    wait_ms(10);
    usb_disconnect();    //Don't enter SLEEP when USB mode
    gpio_disable_init(); 
    palSetLineMode(RENUM,PAL_MODE_INPUT_ANALOG);
    if(kb_mode == KB_MODE_24G)
    wakeup_pack24G = 1;
    
    if(get_plug_mode() == false)
    {
        palSetLineMode(A11,PAL_MODE_INPUT_ANALOG);
        palSetLineMode(A12,PAL_MODE_INPUT_ANALOG);
    }
//开关设置

    setPinInput(BLE);
    setPinInput(TwoMode);
    palEnableLineEvent(BLE, PAL_EVENT_MODE_BOTH_EDGES);
    palEnableLineEvent(TwoMode, PAL_EVENT_MODE_BOTH_EDGES);
    //旋钮唤醒
    setPinInput(encoder_left);
    setPinInput(encoder_right);
    palEnableLineEvent(encoder_left, PAL_EVENT_MODE_RISING_EDGE);
    palEnableLineEvent(encoder_right, PAL_EVENT_MODE_RISING_EDGE);
#if (DIODE_DIRECTION == ROW2COL)

    pin_t col_pins[] = MATRIX_COL_PINS;

    for(uint8_t x=0; x<MATRIX_COLS; x++)
    {
        pin_t pin;
        pin = col_pins[x];
        if (pin != NO_PIN) 
        {
            setPinOutput(pin);
            writePinLow(pin);
        }
    }

    const long unsigned int row_pins[] = MATRIX_ROW_PINS;
    for(uint8_t x=0; x<MATRIX_ROWS; x++)
    { 
        pin_t pin;
        pin = row_pins[x];
        if (pin != NO_PIN) 
        {
            setPinInputHigh(pin);
            palEnableLineEvent(pin, PAL_EVENT_MODE_FALLING_EDGE);
        }
    }
   palEnableLineEvent(PLUG_IN, PAL_EVENT_MODE_BOTH_EDGES);
#else
    const long unsigned int row_pins[] = MATRIX_ROW_PINS;
    for(uint8_t x=0; x<MATRIX_ROWS; x++)
    { 
        pin_t pin;
        pin = row_pins[x];
        if (pin != NO_PIN) 
        {
            setPinOutput(pin);
            writePinLow(pin);
        }
    }

    const long unsigned int col_pins[] = MATRIX_COL_PINS;
    for(uint8_t x=0; x<MATRIX_COLS; x++)
    //for(uint8_t x=0; x<9; x++)
    {
        pin_t pin;
        pin = col_pins[x];
        setPinInputHigh(pin);
        palEnableLineEvent(pin, PAL_EVENT_MODE_BOTH_EDGES);
    }
#endif

 
    ADC1->CR2 &= ~ADC_CR2_ADON;   //0.9ma
    
    //writePinLow(DRIVER_1_EN); 
    /* going to anabiosis*/
    //chSysLock();
    PWR->CR |= (1<<0)|(1<< 10) |(1<< 11)|(3<<18);
    /* Clear Wake-up flag */
    PWR->CR |= PWR_CR_CWUF | PWR_CR_CSBF;

    /* Set SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    //}
    /* Request Wait For Interrupt */
    __WFI();
     SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
     //增加特殊按键消抖处理

#ifdef  mk637_special_isr
    special_key_debounce();
#endif
//休眠防抖

    swif_debounce();
    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;


    setPinInput(BLE);
    setPinInput(TwoMode);   
    setPinOutput(RENUM);
    writePinHigh(RENUM);
    gpio_init();
    stm32_clock_init();
    init_usb_driver(&USB_DRIVER); //Should not enter SLEEP when USB mode                //Should not enter SLEEP when USB mode
    matrix_init();
    ws2812_init();
    adc_init(); 


     get_mode();
    if(kb_mode == KB_MODE_BLE)
    {
        WIRELESS_START(last_wireless_mode);
    }
    if(kb_mode == KB_MODE_24G)
    {
        WIRELESS_START(4);
    }
    rgb_wireless_timer = timer_read32();
    first_sleep_timer = timer_read32();
    packet_send = 0;
    //配对超时和回连超时计时清0
    blink_ble_24g_timer = 0; //蓝牙回连超时     配对也是这个  
    pair_succeed_timer = timer_read32();
    //重新计时连接时间  
    blink_ble_24g_timer = timer_read32(); //重新计时回连时间
    flag32.mode_one = 1;
    blink_counter  = 0;
    ble24G_counter = 0;
    //低电
    ble24G_period = 5;
    pair_timeout = 20000;
  //  BT_Switch_Flag = 1;
    power_counter = 0;      
    loop10hz_token = defer_exec(LOOP_10HZ_PERIOD, loop_10Hz, NULL);
 }



uint8_t sleep_first_flag;
uint16_t testime_flag;
void sleep_mode(void)
{

    //软关机休眠
      if((false == get_plug_mode() ) && (temp == 0 ))
      {

            POWER_EnterSleep(); 
            BT_24G_Shine = 0;   
      }
   //回连超时休眠
    if (!wireless_connected  &&((kb_mode == KB_MODE_BLE ) || (kb_mode == KB_MODE_24G) ) ) 
    {

        if(timer_elapsed32(blink_ble_24g_timer) >= pair_timeout)
        {
            //增加滅燈操作
            rgblight_set_layer_state(6,false);
            rgblight_set_layer_state(3,false);
            rgblight_set_layer_state(4,false);
            rgblight_set_layer_state(5,false);
            rgblight_set_layer_state(7,false);
            rgblight_set_layer_state(8,false);
            POWER_EnterSleep();  
            BT_24G_Shine = 0;  
        }
    }


    if (wireless_connected)   //DO IT WHEN WIRELESS MODE
    {
//测试如果改成10分钟  600s   580s   
        if(timer_elapsed32(first_sleep_timer) >= 8000  && (packet_send < 15) ) 
        {

            first_sleep_timer = timer_read32();
            packet_send++;
            for (int i=0;i<5;i++)
            {
                uart_write(0x00);
            }       

        }

        if(timer_elapsed32(rgb_wireless_timer) > RGB_MATRIX_WIRELESS_TIME)
        {
         
            config_time_alarm();
            POWER_EnterSleep_First();
            BT_24G_Shine = 0;  
            exti_stop_config();
            //如果唤醒源不是按键唤醒，才进去下面
            if(exti_flag)
            {
                exti_flag = 0;
               POWER_EnterSleep();
               BT_24G_Shine = 0;  
            }
            else
            {

              gpio_init();
              sleep_first_flag = 1;
            } 
        }  
    }



    //有线跟随休眠
    if(USB_DRIVER.state == USB_SUSPENDED && kb_mode == KB_MODE_USB)
    { 
        if(usb_suspend_flag == 0)   //USB之前时ACTIVE的状态
        {
            usb_suspend_time = timer_read32();
            usb_suspend_flag = 1;   //USB之前是ACTIVE,现在是SUSPENED
        }

    }
    else if(USB_DRIVER.state == USB_ACTIVE && kb_mode == KB_MODE_USB)
    {
        usb_suspend_flag = 0;   //USB是ACTIVE的状态
      //   usb_suspend_time = timer_read32();   
    }  
    else
    {
        usb_suspend_flag = 2;   
    }

    if(timer_elapsed32(usb_suspend_time) > 500)
    {
       if(get_plug_mode() == true && kb_mode == KB_MODE_USB )
        {
            if(usb_suspend_flag == 1)
            {
     
                usb_suspend_power_down(); 
                BT_24G_Shine = 0;  
                usb_wakeup_flag = true;
            }
        }
    }

    
    if((blink_counter == 5) && (Sleep_First_PressFlag))
    {
        Sleep_First_PressFlag = 0;
        init_usb_driver(&USB_DRIVER); //Should not enter SLEEP when USB mode
    } 










//   if (get_plug_mode() == true && kb_mode == KB_MODE_USB)
//   {

//      if (timer_elapsed32(usb_suspend_time) > 500) {
//          if (usb_suspend_flag == 1) {
//                 // usb_suspend_power_down();
//                 sleep_shutled_flag = 1;
//             }
//      }
//      if (timer_elapsed32(usb_suspend_time) > 1500) {
//          if (usb_suspend_flag == 1) {
//                 usb_suspend_power_down();
//                 sleep_shutled_flag = 0;
//          //       rgblight_set_layer_state(1,false);
//                 // ws2812_init();
//                 // rgblight_set_layer_state(4,false);
//             }
//      }
//   }



    //2.4G跟随休眠
    if(suspend == true)
    {
 
       POWER_EnterSleep(); 
         BT_24G_Shine = 0;   
       suspend = false;
    }


}





void adc_test(void)
{

    if((timer_elapsed32(battery_test_time) > 30000))
    {
        adc_value = get_adc_value();    //获取adc值
        adc_vref =  get_adc_vref();
        battery_value = (adc_value*1764/adc_vref);
        temp = batt_level();
        battery_test_time = timer_read32();



    //电量一点点增加或者减少

        if(get_plug_mode() == true)     //插usb
        {
            if(temp > last_temp +1)     //暴涨超2%
            {
                second_temp = temp;
                if(last_temp != second_temp)
                {
                    last_temp++;
                    temp = last_temp;
                }
            }
            else if(last_temp == temp || temp == last_temp +1)  //电量正常不加或是只增长1%
            {
                last_temp = temp;
            }
        }
        else
        {
            if(last_temp == temp || temp == last_temp -1)   //电量正常不减少或是只减少1%
            {
                last_temp = temp;
            }
            else if(temp < last_temp -1)    //暴跌超2%
            {
                second_temp = temp;
                if(last_temp != second_temp)
                {
                    last_temp--;
                    temp = last_temp;
                }
            }
            else if(temp > last_temp)   //这是开灯测的电量，突然关灯会导致adc值暴涨
            {
                temp = last_temp;
            }
        }
    }
}
void ble_send_batt(void)
{
    if(kb_mode==KB_MODE_BLE && wireless_connected) //no battery info when 24G or connecting
    {
        if(ble_flag == 0)
        {             

             battery_Flag = 1;
            if(!keyboard_Idle)
            {
              battery_Flag = 0;
              sc_ble_battary(temp); 
            }
                       

            ble_flag = 1;
            ble_bat_timer = timer_read32();
        }
        
        if(timer_elapsed32(ble_bat_timer) > 30000)
        {
            if(get_plug_mode() == true)     //插usb
            {
                if(temp > last_temp +1)     //暴涨超2%
                {
                    second_temp = temp;
                    if(last_temp != second_temp)
                    {
                        last_temp++;
                        temp = last_temp;
                    }
                }
                else if(last_temp == temp || temp == last_temp +1)  //电量正常不加或是只增长1%
                {
                    last_temp = temp;
                }
            }
            else
            {
                if(last_temp == temp || temp == last_temp -1)   //电量正常不减少或是只减少1%
                {
                    last_temp = temp;
                }
                else if(temp < last_temp -1)    //暴跌超2%
                {
                    second_temp = temp;
                    if(last_temp != second_temp)
                    {
                        last_temp--;
                        temp = last_temp;
                    }
                }
                else if(temp > last_temp)   //这是开灯测的电量，突然关灯会导致adc值暴涨
                {
                    temp = last_temp;
                }
            }
             battery_Flag = 1;
            if(!keyboard_Idle)
            {
              battery_Flag = 0;
              sc_ble_battary(temp); 
            }

            ble_bat_timer = timer_read32();
        }       
    }
}





static uint16_t count_flag;
uint8_t Rtc_Config_Api(void)
{
   rcu_backup_clock_Enable();//备份区域的时钟使能
   rcu_power_clock_Enable();//电源管理时钟使能
   backup_write_Enable();//备份区允许访问
  // bkp_reset();//备份域复位
   rcu_inlowclock_on();//使能内部低速时钟
 //  wait_inlowclock_sta();//等待内部时钟稳定
   while(!(RCC->CSR  & RCC_BDCR_LSERDY) &&  count_flag<250)
   {
     count_flag++;
     wait_ms(10); 
   }
   if(count_flag >=250)
   {
    return 1;
   }
   rcu_clock_source();  //RTC时钟源选择
   RCU_Clock_enable();//使能RTC时钟
   rtc_register_sync_wait();//等待寄存器域APB1时钟同步
    RTC->CRH |= RTC_CRH_ALRIE;
    while(!(RTC->CRL & RTC_CRL_RTOFF));  //RTC operation OFF
    RTC->CRL |= RTC_CRL_CNF;       //Configuration Flag  
    RTC->CRL |= RTC_CRL_CNF;
    RTC->PRLL =  0x9C3F;//0x9C3F//0x9C3F;//0x9C3F;  //RTC Prescaler Reload Value Low 
    RTC->CRL &= ~RTC_CRL_CNF;       //clear reset Flag
    while(!(RTC->CRL & RTC_CRL_RTOFF));  //RTC operation OFF
    rcu_backup_clock_Enable();//备份区域的时钟使能
    rcu_power_clock_Enable();//电源管理时钟使能
    backup_write_Enable();//备份区允许访问
    RTC->CRL |= RTC_CRL_CNF;       //Configuration Flag  
    RTC->CNTL = 0x00;
    RTC->CRL &= ~RTC_CRL_CNF; 
    while(!(RTC->CRL & RTC_CRL_RTOFF));  //RTC operation OFF
    rcu_backup_clock_Enable();//备份区域的时钟使能
    rcu_power_clock_Enable();//电源管理时钟使能
    backup_write_Enable();//备份区允许访问
    RTC->CRL |= RTC_CRL_CNF;       //Configuration Flag  
    RTC->ALRH = 0;
    RTC->ALRL = Second_level_sleeptime;//0x483;//0x483;//5分钟//0x474;//0x4b0;   //时间时15s
    RTC->CRL &= ~RTC_CRL_CNF; 
    while(!(RTC->CRL & RTC_CRL_RTOFF));  //RTC operation OFF 
    RTC->CRL &= ~RTC_CRL_CNF;
    exti_stop_config();
    return 0;
}
 


uint8_t  isr_specal_Trig;
//休眠防抖
void swif_debounce(void)
{
    uint8_t isr_source[MATRIX_ROWS];
    uint8_t i;
    const long unsigned int row_pins[] = MATRIX_ROW_PINS;
    //休眠防抖处理,前提是中断源是按键唤醒
    if(isr_Trig)
    {
        while(1)
        {         
            wait_ms(5); 
            //读6次row口的电平
            for(uint8_t j = 0; j< 10; j++)
            {
                for(i = 0; i < MATRIX_ROWS; i++)
                {     
                    isr_source[i] = readPin(row_pins[i]);
                    if(!isr_source[i])
                    break;
                }
                if(i < MATRIX_ROWS)   
                {
                    break;
                }
            }


            if(i < MATRIX_ROWS)   
            {
                break;
            }
            else
            {
                pin_t col_pins[] = MATRIX_COL_PINS;
                for(uint8_t x=0; x<MATRIX_COLS; x++)
                {
                    pin_t pin;
                    pin = col_pins[x];
                    if (pin != NO_PIN) 
                    {
                        setPinOutput(pin);
                        writePinLow(pin);
                    }
                }

             
                for(uint8_t x=0; x<MATRIX_ROWS; x++)
                { 
                    pin_t pin;
                    pin = row_pins[x];
                    if (pin != NO_PIN) 
                    {
                        setPinInputHigh(pin);
                        palEnableLineEvent(pin, PAL_EVENT_MODE_FALLING_EDGE);
                    }
                }
                PWR->CR |= (1<<0)|(1<< 10) |(1<< 11)|(3<<18);
                PWR->CR |= PWR_CR_CWUF | PWR_CR_CSBF;
                SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
                __WFI();
                SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

            }    
        }
          //  test_count = 100;
    }
 
    EXTI->IMR  &= ~0xffff;   
    EXTI->EMR  &= ~0xffff;
    EXTI->RTSR &= ~0xffff;
    EXTI->FTSR &= ~0xffff;
    isr_Trig = 0;
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


