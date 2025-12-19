#!/bin/bash

# generate yolox model
stedgeai generate --no-inputs-allocation --no-outputs-allocation --model st_yoloxn_d067_w025_416_int8.tflite --target stm32n6 --st-neural-art default@user_neuralart.json
cp st_ai_output/network_ecblobs.h .
cp st_ai_output/network.c .
cp st_ai_output/stai_network.c .
cp st_ai_output/stai_network.h .
cp st_ai_output/network_atonbuf.xSPI2.raw network_data.xSPI2.bin
arm-none-eabi-objcopy -I binary network_data.xSPI2.bin --change-addresses 0x70380000 -O ihex network_data.hex
# remove tmpdir
rm -rf st_ai_output st_ai_ws
