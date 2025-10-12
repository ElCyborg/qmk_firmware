#pragma once
#include "quantum.h"
//一级休眠时间
#define  RGB_MATRIX_WIRELESS_TIME   600000
//二级休眠时间
#define  Second_level_sleeptime   0x4C4 // 0x672     0x131 5分钟   25分钟 0x5f5    20分钟4c4 

//中断源配置
#define isr_soruce0  
#define isr_soruce1 
#define isr_soruce2  
#define isr_soruce3  
#define isr_soruce4  

//配置侧灯灯效刷新，由于不确定第二路灯效的RGB的具体位置，换项目就重新增加，底层之前的不要去改
#define Effect_MK637

//配置侧灯分别控制模式和颜色
#define Right_Switch_MK637

#define MK637_Flash_Store
// #define mk637_special_isr

//引脚配置
//usb相关引脚
#define PLUG_IN 		    B9     //插入检测
#define BT_TOG_PIN          B1
#define RENUM   A8         

//灯控引脚
// #define ARGB_LEFT_EN  B4
 #define ARGB_LEFT_EN  A5
//晶振引脚
#define OSC_IN D0
#define OSC_OUT D1

//模式

#define BLE C15
// #define BLE2 A13
// #define BLE3 A6
#define TwoMode C14

//系统切换
// #define SYS_SW A7
//旋钮引脚
#define  encoder_left A7
#define  encoder_right A6

extern uint8_t shut_up_Flag;



/************************时间定义************************/
#define PAIR_SUCCEED_TIME           2800	//配对成功指示灯常亮时间
#define BLE_24G_TIMEOUT             3000    //蓝牙2.4g长按时间
#define RESET_TIMEOUT               3000    //复位长按时间  //ms 


extern uint8_t  isr_specal_Trig;

enum custom_keycodes {
#ifdef VIA_ENABLE
//    KC_USB = QK_USER,
    KC_USB = QK_KB_0,
#else
    KC_USB = SAFE_RANGE,
#endif
  KC_BLE1,
  KC_BLE2,
  KC_BLE3,
  KC_24G,
  KC_BAT,
  KC_RESET,
  KC_WIN,
  KC_WINMODE,
  KC_MACMODE,
  KC_SHUTRGB,
  KC_RRWIN,
  KC_MODEPLUS,
  KC_Mctl,
  KC_Lpad,
  KC_HUEPLS,
  ENC_TG,
  KC_VAL_UP,
  KC_VAL_DN,
  KC_SPD_UP,
  KC_SPD_DN,
  KC_LOCK


};