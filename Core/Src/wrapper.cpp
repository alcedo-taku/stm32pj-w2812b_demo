#include "wrapper.hpp"

#include "stdint.h"
#include "tim.h"
#include <cmath>
//#include <array>

#include "ws2812b.hpp"

/* Pre-Processor Begin */

/* Pre-Processor End */

/* Enum, Struct Begin */

/* Enum, Struct End */

/* Function Prototype Begin */
void set_led(int led_num, int red, int green, int blue);
void set_brightness (int brightness);
void ws2812_send(void);
void send(int green, int red, int blue);

/* Function Prototype End */

/* Variable Begin */
#define MAX_LED 8
#define USE_BRIGHTNESS 1

uint8_t led_data[MAX_LED][4];
uint8_t led_mod[MAX_LED][4];  // for brightness
uint16_t pwmData[(24*MAX_LED)+50];
uint16_t pwmData_singl[24];

int datasentflag = 0;

ws2812b::WS2812B tape_led(&htim1,TIM_CHANNEL_1,MAX_LED);
std::vector<ws2812b::RGB_t> color_rgb = {
    ws2812b::RGB_t{255, 0, 0},
    ws2812b::RGB_t{0, 255, 0},
    ws2812b::RGB_t{0, 0, 255},
    ws2812b::RGB_t{46, 89, 128},
    ws2812b::RGB_t{156, 233, 100},
    ws2812b::RGB_t{102, 0, 235},
    ws2812b::RGB_t{47, 38, 77},
    ws2812b::RGB_t{255, 200, 0}
};

/* Variable End */

void init(void){
    //  send(5, 255, 101);
    //  set_led(0, 255, 0, 0);
    //  set_led(1, 0, 255, 0);
    //  set_led(2, 0, 0, 255);
    //  ws2812_send();
    tape_led.init();
}

void loop(void){
    //    send(5, 255, 101);
//    set_led(0, 255, 0, 0);
//    set_led(1, 0, 255, 0);
//    set_led(2, 0, 0, 255);
//    set_led(3, 46, 89, 128);
//    set_led(4, 156, 233, 100);
//    set_led(5, 102, 0, 235);
//    set_led(6, 47, 38, 77);
//    set_led(7, 255, 200, 0);
//    ws2812_send();

//    tape_led.set_color_rgb(0, 255, 0, 0);
//    tape_led.set_color_rgb(1, 0, 255, 0);
//    tape_led.set_color_rgb(2, 0, 0, 255);
//    tape_led.set_color_rgb(3, 46, 89, 128);
//    tape_led.set_color_rgb(4, 156, 233, 100);
//    tape_led.set_color_rgb(5, 102, 0, 235);
//    tape_led.set_color_rgb(6, 47, 38, 77);
//    tape_led.set_color_rgb(7, 255, 200, 0);
//    tape_led.set_color_rgb(color_rgb);
//    tape_led.set_color_hsv(0, 0, 100, 100);
//    tape_led.set_color_hsv(1, 120, 100, 100);
//    tape_led.set_color_hsv(2, 240, 100, 100);
//    tape_led.send();
//
//    HAL_Delay (100);

    for (int i=0; i<360; i++){
//        set_brightness(i);
//        ws2812_send();
        for (uint8_t j = 0; j < MAX_LED; j++) {
//            tape_led.set_color_hsv(j, 360.0f*j/MAX_LED + i*8, 100, 50); // ゲーミング
//            tape_led.set_color_hsv(j, 360.0f*j/MAX_LED + i, 100, 100.0f*(MAX_LED-j)/MAX_LED); // ゲーミング＋固定明るさグラデーション
            tape_led.set_color_hsv(j, 360.0f*j/MAX_LED + i, 100.0f*(MAX_LED-j)/MAX_LED, 50); // ゲーミング＋固定彩度グラデーション
//            tape_led.set_color_hsv(j, 360.0f*j/MAX_LED + i, 100.0f*(MAX_LED-j)/MAX_LED, 100.0f*(MAX_LED-j)/MAX_LED); // ゲーミング＋固定彩度明度グラデーション

//            // 点が移動していく
//            if (j==(uint8_t)(i/360.0f*MAX_LED)){
//                tape_led.set_color_hsv(j, 360.0f*j/MAX_LED + i, 100.0f*(MAX_LED-j)/MAX_LED, 100.0f*(MAX_LED-j)/MAX_LED); // ゲーミング＋固定彩度明度グラデーション
//            }else{
//                tape_led.set_color_hsv(j, 360.0f*j/MAX_LED + i, 100.0f*(MAX_LED-j)/MAX_LED, 0); // ゲーミング＋固定彩度明度グラデーション
//            }

            // 点（大きさ及びぼやけがある）がなめらかに移動していく
        }
        tape_led.send();
        HAL_Delay (3);
    }

//    for (int i=45; i>=0; i--){
////        set_brightness(i);
////        ws2812_send();
//
//        for (uint8_t j = 0; j < MAX_LED; j++) {
//            tape_led.set_color_hsv(j, 360.0f*j/MAX_LED + i*4, 100, 100);
//        }
//        tape_led.send();
//        HAL_Delay (50);
//    }
}

/* Function Body Begin */


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
//    HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
//    datasentflag = 1;
}

void set_led(int led_num, int red, int green, int blue){
    led_data[led_num][0] = led_num;
    led_data[led_num][1] = green;
    led_data[led_num][2] = red;
    led_data[led_num][3] = blue;
}

void set_brightness (int brightness)  // 0-45
{
#if USE_BRIGHTNESS
    if (brightness > 45) brightness = 45;
    for (int i=0; i<MAX_LED; i++)
    {
        for (int j=1; j<4; j++)
        {
            float angle = 90-brightness;  // in degrees
            angle = angle * M_PI / 180;  // in rad
            led_mod[i][j] = (led_data[i][j])/(tan(angle));
        }
    }
#endif
}

void ws2812_send(void){
    uint32_t indx = 0;
    uint32_t color;
    for (uint8_t i = 0; i < MAX_LED; i++) {
#if USE_BRIGHTNESS
        color = ((led_mod[i][1]<<16) | (led_mod[i][2]<<8) | (led_mod[i][3]));
#else
        color = ((led_data[i][1]<<16) | (led_data[i][2]<<8) | (led_data[i][3]));
#endif
        for (int j = 23; j >= 0; j--) {
            if (color&(1<<j)) pwmData[indx] = 60; // 2/3 of 90
            else pwmData[indx] = 30; // 1/3 of 90
            indx++;
        }
    }

    for (uint8_t i = 0; i < 50; i++) {
        pwmData[indx] = 0;
        indx++;
    }

    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)pwmData, indx);
    while(!datasentflag){};
    datasentflag = 0;
}

void send(int green, int red, int blue){
    uint32_t color = (green<<16) | (red<<8) | blue;
    for (int i = 23; i >= 0; i--) {
        if (color&(1<<i)) pwmData_singl[i] = 60; // 2/3 of 90
        else pwmData_singl[i] = 30; // 1/3 of 90
    }
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)pwmData_singl, 24);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 30);
}
/* Function Body End */
