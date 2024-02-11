#include "wrapper.hpp"

#include "stdint.h"
#include "tim.h"
#include <cmath>

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

/* Variable End */

void init(void){
    //  send(5, 255, 101);
    //  set_led(0, 255, 0, 0);
    //  set_led(1, 0, 255, 0);
    //  set_led(2, 0, 0, 255);
    //  ws2812_send();
}

void loop(void){
    //    send(5, 255, 101);
    set_led(0, 255, 0, 0);
    set_led(1, 0, 255, 0);
    set_led(2, 0, 0, 255);
    set_led(3, 46, 89, 128);
    set_led(4, 156, 233, 100);
    set_led(5, 102, 0, 235);
    set_led(6, 47, 38, 77);
    set_led(7, 255, 200, 0);

    for (int i=0; i<46; i++){
        set_brightness(i);
        ws2812_send();
        HAL_Delay (50);
    }

    for (int i=45; i>=0; i--){
        set_brightness(i);
        ws2812_send();
        HAL_Delay (50);
    }
}

/* Function Body Begin */


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
    HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
    datasentflag = 1;
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
