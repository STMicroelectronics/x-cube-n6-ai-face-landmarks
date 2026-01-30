/**
  ******************************************************************************
  * @file    lcd_conf.h
  * @author  MCD Application Team
  * @brief   This file contains all the functions prototypes for the ili9341_reg.c
  *          driver.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __LCD_CONF_H__
#define __LCD_CONF_H__

#include "stm32n6xx_hal.h"

#define LCD_RST_LOW()                       WRITE_REG(GPIOC->BRR, GPIO_PIN_10)
#define LCD_RST_HIGH()                      WRITE_REG(GPIOC->BSRR, GPIO_PIN_10)

#define LCD_CS_LOW()                        WRITE_REG(GPIOD->BRR, GPIO_PIN_12)
#define LCD_CS_HIGH()                       WRITE_REG(GPIOD->BSRR, GPIO_PIN_12)

#define LCD_DC_LOW()                        WRITE_REG(GPIOD->BSRR, GPIO_PIN_5)
#define LCD_DC_HIGH()                       WRITE_REG(GPIOD->BRR, GPIO_PIN_5)

#endif
