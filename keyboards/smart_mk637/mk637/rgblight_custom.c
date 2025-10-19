#include "quantum.h"
#include "sc_ws2812.h"
#include "print.h"

#ifdef RGBLIGHT_ENABLE

void setleds_custom(rgb_led_t *led, uint16_t led_num) {
    
    // sc_ws2812_setleds(led,led_num);
}
void flush_custom(void) {
    //    aw9523b_update_pwm_buffers(AW9523B_ADDR);
    //    ws2812_flush();
}
void set_color_custom(int index, uint8_t red, uint8_t green, uint8_t blue) {
//    print("ehre\n");
//    if (index < AW9523B_RGB_NUM) {
//        aw9523b_set_color(index, red, green, blue);
//
//    } else {
//        ws2812_set_color(index, red, green, blue);
//    }
}
void set_color_all_custom(uint8_t red, uint8_t green, uint8_t blue) {
//    for (int i = 0; i < RGBLIGHT_LED_COUNT; i++) {
//        set_color_custom(i, red, green, blue);
//    }
}


const rgblight_driver_t rgblight_driver = {
    .init = ws2812_init, 
    .set_color = set_color_custom, 
    .set_color_all = set_color_all_custom, 
    .flush = flush_custom
};

#endif  // RGBLIGHT_ENABLE