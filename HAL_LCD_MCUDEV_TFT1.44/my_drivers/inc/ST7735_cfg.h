/*
 * ST7735_cfg.h
 *
 *  Created on: Sep 18, 2024
 *      Author: maciej
 */

#ifndef MY_DRIVERS_ST7735_INC_ST7735_CFG_H_
#define MY_DRIVERS_ST7735_INC_ST7735_CFG_H_

#include "main.h"
#define ST7735_SPI_PORT hspi2 //hspi1, hspi2, hspi3...
//#define USE_SPI_DMA     //if used DMA for SPI bus

/*
*	USE ALL FUNCT (backligh + reset pin) -- NOT IMPLEMENTET IN ST_SMART -- uncomment OPTIONAL
*/
//#define USE_ALL_FUNC

//#define ST7735_1_8_DEFAULT_ORIENTATION  // AliExpress/eBay 1.8" display, default orientation
//#define ST7735S_1_8_DEFAULT_ORIENTATION   // WaveShare ST7735S-based 1.8" display, default orientation
#define ST7735_1_44_DEFAULT_ORIENTATION   // 1.44" display, default orientation
//#define ST7735_MINI_DEFAULT_ORIENTATION   // mini 160x80 display (it's unlikely you want the default orientation)

//Port and pin connected signal 'RES' (reset) ST7735 display
#ifdef USE_ALL_FUNC
#ifndef ST7735_RES_Pin
#define ST7735_RES_Pin      GPIO_PIN_15
#endif
#endif

#ifndef ST7735_RES_GPIO_Port
#define ST7735_RES_GPIO_Port  GPIOB
#endif
//Port and pin connected signal 'DC' (data or command) ST7735 display
#ifndef ST7735_DC_Pin
#define ST7735_DC_Pin       GPIO_PIN_7
#endif
#ifndef ST7735_DC_GPIO_Port
#define ST7735_DC_GPIO_Port   GPIOC
#endif
//Port and pin connected signal 'CS' (chip select) ST7735 display
#ifndef ST7735_CS_Pin
#define ST7735_CS_Pin       GPIO_PIN_6
#endif
#ifndef ST7735_CS_GPIO_Port
#define ST7735_CS_GPIO_Port   GPIOC
#endif
//Port and pin connected signal 'BL' (back light) ST7735 display
#ifdef USE_ALL_FUNC
#ifndef ST7735_BL_Pin
#define ST7735_BL_Pin     GPIO_PIN_15
#endif
#ifndef ST7735_BL_GPIO_Port
#define ST7735_BL_GPIO_Port   GPIOB
#endif
#endif


#endif /* MY_DRIVERS_ST7735_INC_ST7735_CFG_H_ */
