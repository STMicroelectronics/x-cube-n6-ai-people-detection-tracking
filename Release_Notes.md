# Release Notes for x-cube-n6-ai-people-detection-tracking Application

## Purpose

Computer Vision application demonstrating the deployment of object detections models on STM32N6570-DK and NUCLEO-N657X0-Q boards.

This application is prebuilt with a people detection model, "YoloX" for STM32N6570-DK and "TinyYOLOv2" for NUCLEO-N657X0-Q.

## Key Features

- Multi-threaded application flow (FreeRTOS)
- Tracking and box filtering
- NPU accelerated quantized AI model inference
- Dual DCMIPP pipes
- DCMIPP crop, decimation, downscale
- LTDC dual-layer implementation
- DCMIPP ISP usage
- Dev mode
- Boot from External Flash

## Software components

| Name                          | Version             | Release notes
|-----                          | -------             | -------------
| STM32Cube.AI runtime          | 11.0.0              |
| Camera Middleware             | v1.5.0              | [release notes](Lib/Camera_Middleware/Release_Notes.md)
| lib_vision_models_pp Library  | v0.13.0             | [release notes](Lib/lib_vision_models_pp/lib_vision_models_pp/README.md)
| screenl                       | v3.0.0              | [release notes](Lib/screenl/Release_Notes.html)
| tracker                       | v1.0.1              | [release notes](Lib/tracker/Release_Notes.html)
| post process wrapper          | v1.0.9              | [release notes](Lib/ai-postprocessing-wrapper/Release_Notes.html)
| CMSIS                         | V6.2.0              | [release notes](STM32Cube_FW_N6/Drivers/CMSIS/Documentation/index.html)
| STM32N6xx CMSIS Device        | V1.3.0              | [release notes](STM32Cube_FW_N6/Drivers/CMSIS/Device/ST/STM32N6xx/Release_Notes.html)
| STM32N6xx HAL/LL Drivers      | V1.3.0              | [release notes](STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Release_Notes.html)
| STM32N6570-DK BSP Drivers     | V1.3.0              | [release notes](STM32Cube_FW_N6/Drivers/BSP/STM32N6570-DK/Release_Notes.html)
| STM32N6xx_Nucleo BSP Drivers  | V1.2.0              | [release notes](STM32Cube_FW_N6/Drivers/BSP/STM32N6xx_Nucleo/Release_Notes.html)
| BSP Component aps256xx        | V1.0.6              | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx/Release_Notes.html)
| BSP Component Common          | V7.3.0              | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/Common/Release_Notes.html)
| BSP Component mx66uw1g45g     | V1.1.0              | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/mx66uw1g45g/Release_Notes.html)
| BSP Component rk050hr18       | V1.0.1              | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/rk050hr18/Release_Notes.html)
| FreeRTOS kernel               | v10.6.2             | [release notes](Lib/FreeRTOS/Source/History.txt)
| Azure RTOS USBX               | V6.4.0              | [release notes](STM32Cube_FW_N6/Middlewares/ST/usbx/README.md)
|                               | ST modified 251022  | [ST release notes](STM32Cube_FW_N6/Middlewares/ST/usbx/st_readme.txt)
| Fonts Utility                 | V2.0.3              | [release notes](STM32Cube_FW_N6/Utilities/Fonts/Release_Notes.html)
| lcd Utility                   | V2.2.0              | [release notes](STM32Cube_FW_N6/Utilities/lcd/Release_Notes.html)
| ai fsbl                       | v1.4.0              | [release notes](FSBL/Release_Notes.md)

## Update history

### V2.2.0 / January 2026

- Update the STM32CubeN6 Firmware Package to version 1.3.0.
- Update STEdgeAI to version 3.0.0.
- Switch to STEdgeAI `st-ai` API.
- Add STEVAL-1943-MC1 camera module support.

### V2.1.0 / August 2025

- Update the STM32CubeN6 Firmware Package to version 1.2.0.
- Update STEdgeAI to version 2.2.0.

### V2.0.0 / May 2025

- Replace Threadx by FreeRTOS
- Add box tracking and filtering
- Add support of NUCLEO-N657X0-Q
- Rename application from x-cube-n6-ai-people-detection to x-cube-n6-ai-people-detection-tracking

### V1.0.0 / December 2024

Initial Version
