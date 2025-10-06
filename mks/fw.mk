FW_REL_DIR := STM32Cube_FW_N6

C_SOURCES_FW += $(FW_REL_DIR)/Drivers/CMSIS/Device/ST/STM32N6xx/Source/Templates/system_stm32n6xx_fsbl.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_cortex.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_dcmipp.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_dma2d.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_gpio.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_i2c.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_i2c_ex.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_ltdc.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_ltdc_ex.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_rif.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_ramcfg.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_cacheaxi.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_pwr.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_pwr_ex.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_rcc.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_rcc_ex.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_xspi.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_bsec.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_uart.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_exti.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_tim.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_pcd.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_pcd_ex.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_ll_usb.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_spi.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_dma.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_dma_ex.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_gpu2d.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_icache.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_gfxmmu.c
C_SOURCES_FW += $(FW_REL_DIR)/Utilities/lcd/stm32_lcd.c
ifeq ($(call is_nucleo),1)
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/BSP/STM32N6xx_Nucleo/stm32n6xx_nucleo.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/BSP/STM32N6xx_Nucleo/stm32n6xx_nucleo_bus.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/BSP/STM32N6xx_Nucleo/stm32n6xx_nucleo_xspi.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/BSP/Components/mx25um51245g/mx25um51245g.c
else
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/BSP/STM32N6570-DK/stm32n6570_discovery.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/BSP/STM32N6570-DK/stm32n6570_discovery_bus.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/BSP/STM32N6570-DK/stm32n6570_discovery_lcd.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/BSP/STM32N6570-DK/stm32n6570_discovery_xspi.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/BSP/Components/aps256xx/aps256xx.c
C_SOURCES_FW += $(FW_REL_DIR)/Drivers/BSP/Components/mx66uw1g45g/mx66uw1g45g.c
endif

C_INCLUDES_FW += -I$(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Inc
C_INCLUDES_FW += -I$(FW_REL_DIR)/Drivers/STM32N6xx_HAL_Driver/Inc/Legacy
C_INCLUDES_FW += -I$(FW_REL_DIR)/Drivers/CMSIS/Device/ST/STM32N6xx/Include
C_INCLUDES_FW += -I$(FW_REL_DIR)/Drivers/CMSIS/Include
C_INCLUDES_FW += -I$(FW_REL_DIR)/Drivers/CMSIS/DSP/Include
C_INCLUDES_FW += -I$(FW_REL_DIR)/Drivers/BSP/Components/Common
ifeq ($(call is_nucleo),1)
C_INCLUDES_FW += -I$(FW_REL_DIR)/Drivers/BSP/STM32N6xx_Nucleo
else
C_INCLUDES_FW += -I$(FW_REL_DIR)/Drivers/BSP/STM32N6570-DK
endif
C_INCLUDES_FW += -I$(FW_REL_DIR)/Utilities/lcd

C_SOURCES += $(C_SOURCES_FW)
C_INCLUDES += $(C_INCLUDES_FW)
