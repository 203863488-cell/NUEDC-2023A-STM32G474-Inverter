#ifndef BUTTON_AND_LED_H
#define BUTTON_AND_LED_H

#include <stdint.h>
#include "main.h"

static inline uint8_t Button_GetButton1State(void){
    return (uint8_t)(HAL_GPIO_ReadPin(Button1_GPIO_Port, Button1_Pin) == GPIO_PIN_RESET);
}
static inline uint8_t Button_GetButton2State(void){
    return (uint8_t)(HAL_GPIO_ReadPin(Button2_GPIO_Port, Button2_Pin) == GPIO_PIN_RESET);
}
static inline uint8_t Button_GetButton3State(void){
    return (uint8_t)(HAL_GPIO_ReadPin(Button3_GPIO_Port, Button3_Pin) == GPIO_PIN_RESET);
}
static inline uint8_t Button_GetButton4State(void){
    return (uint8_t)(HAL_GPIO_ReadPin(Button4_GPIO_Port, Button4_Pin) == GPIO_PIN_RESET);
}
static inline uint8_t Button_GetEncoderButtonState(void){
    return (uint8_t)(HAL_GPIO_ReadPin(Encoder_button_GPIO_Port, Encoder_button_Pin) == GPIO_PIN_RESET);
}

static inline void LED_SetLED1State(uint8_t state){
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
static inline void LED_SetLED2State(uint8_t state){
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
static inline void LED_SetLED3State(uint8_t state){
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
static inline void LED_SetLED4State(uint8_t state){
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}


#endif
