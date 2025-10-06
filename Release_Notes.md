# Release Notes for x-cube-n6-ai-face-landmarks Application

## Purpose

Computer Vision application demonstrating the deployment of several object detections models in serial on STM32N6570-DK
and NUCLEO-N657X0-Q boards.
The use case chosen is multi-faces landmark detection. It is made up of two models that are executing in serial.

## Key Features

- Multi-threaded application flow (FreeRTOS)
- NPU accelerated quantized AI model inference
- GPU2D usage to perform resize + rotation transformation
- Execute multiple models serially
- Dual DCMIPP pipes
- DCMIPP crop, decimation, downscale
- LTDC dual-layer implementation
- DCMIPP ISP usage
- Dev mode
- Boot from External Flash

## Software components

| Name                          | Version             | Release notes
|-----                          | -------             | -------------
| STM32Cube.AI runtime          | 10.2.0              |
| Camera Middleware             | v1.4.3              | [release notes](Lib/Camera_Middleware/Release_Notes.md)
| Ipl Library                   | v1.1.0              | [release notes](Lib/ipl/README.md)
| lib_vision_models_pp Library  | v0.12.0             | [release notes](Lib/lib_vision_models_pp/lib_vision_models_pp/README.md)
| screenl                       | v3.0.0              | [release notes](Lib/screenl/Release_Notes.html)
| post process wrapper          | v1.0.8              | [release notes](Lib/ai-postprocessing-wrapper/Release_Notes.html)
| CMSIS                         | v5.9.0              | [release notes](STM32Cube_FW_N6/Drivers/CMSIS/Documentation/index.html)
| STM32N6xx CMSIS Device        | v1.2.0              | [release notes](STM32Cube_FW_N6/Drivers/CMSIS/Device/ST/STM32N6xx/Release_Notes.html)
| STM32N6xx HAL/LL Drivers      | v1.2.0              | [release notes](STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Release_Notes.html)
| STM32N6570-DK BSP Drivers     | v1.2.0              | [release notes](STM32Cube_FW_N6/Drivers/BSP/STM32N6570-DK/Release_Notes.html)
| BSP Component aps256xx        | v1.0.6              | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx/Release_Notes.html)
| BSP Component Common          | v7.3.0              | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/Common/Release_Notes.html)
| BSP Component mx66uw1g45g     | v1.1.0              | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/mx66uw1g45g/Release_Notes.html)
| BSP Component rk050hr18       | v1.0.1              | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/rk050hr18/Release_Notes.html)
| FreeRTOS kernel               | v10.6.2             | [release notes](Lib/FreeRTOS/Source/History.txt)
| Azure RTOS USBX               | v6.4.0              | [release notes](STM32Cube_FW_N6/Middlewares/ST/usbx/README.md)
|                               | ST modified 240906  | [ST release notes](STM32Cube_FW_N6/Middlewares/ST/usbx/st_readme.txt)
| Fonts Utility                 | v2.0.3              | [release notes](STM32Cube_FW_N6/Utilities/Fonts/Release_Notes.html)
| lcd Utility                   | v2.2.0              | [release notes](STM32Cube_FW_N6/Utilities/lcd/Release_Notes.html)
| Nema SDK                      | v1.3.0              |

## Update history

### v1.0.0 / September 2025

Initial Version
