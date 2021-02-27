#ifndef SERVSENSOR_H
#define SERVSENSOR_H

#include "stm32f103xe.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern IWDG_HandleTypeDef hiwdg;

/* elesy begin ******************************************************** */
/* ******************************************************************** */

extern const char version[][16];

#define MODNAME			"servsens"	// Имя модуля
#define SOFTNAME		"ss"		// Имя прошивки
#define VERSION			version[0]	// Версия прошивки
#define REALDATA		version[1]	// Дата последнего изменения

/* ******************************************************************** */
/* elesy begin ******************************************************** */


/* supply begin ******************************************************* */
/* ******************************************************************** */

#define SUPPLY_STARTUP_DELAY_MS	10

#define SUPPLY_P_PORT 			GPIOA
#define SUPPLY_5V_P_PIN			GPIO_PIN_7
#define SUPPLY_3_3V_PIN			GPIO_PIN_6
#define SUPPLY_P_CLOCK_CMD(en) 	if (en) __HAL_RCC_GPIOA_CLK_ENABLE();

#define SUPPLY_EN_5_PORT 			GPIOD
#define SUPPLY_EN_5_PIN 			GPIO_PIN_13
#define SUPPLY_EN_5_CLOCK_CMD(en) 	if (en) __HAL_RCC_GPIOD_CLK_ENABLE();

/* ******************************************************************** */
/* supply begin ******************************************************* */


/* accel begin ******************************************************** */
/* ******************************************************************** */
#define ACCEL_TIMER		TIM1
/* ******************************************************************** */
/* accel begin ******************************************************** */

extern void main_init(void);
extern void main_loop(void);

#endif // SERVSENSOR_H
