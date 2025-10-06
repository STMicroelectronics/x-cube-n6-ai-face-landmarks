#!/bin/bash

stedgeai generate --no-inputs-allocation --name face_detector --model blazeface_front_128_quant_pc_ff_od_wider_face.tflite --target stm32n6 --st-neural-art face_detector@user_neuralart.json --input-data-type uint8 --output-data-type int8
cp st_ai_output/face_detector_ecblobs.h .
cp st_ai_output/face_detector.c .
cp st_ai_output/face_detector_atonbuf.xSPI2.raw st_ai_output/face_detector_data.xSPI2.bin
arm-none-eabi-objcopy -I binary st_ai_output/face_detector_data.xSPI2.bin --change-addresses 0x70380000 -O ihex face_detector_data.hex

stedgeai generate --name face_landmark --model face_landmarks_v1_192_int8_pc.onnx --target stm32n6 --st-neural-art face_landmark@user_neuralart.json --input-data-type uint8 --output-data-type float32 --inputs-ch-position chlast --no-inputs-allocation
cp st_ai_output/face_landmark_ecblobs.h .
cp st_ai_output/face_landmark.c .
cp st_ai_output/face_landmark_atonbuf.xSPI2.raw st_ai_output/face_landmark_data.xSPI2.bin
arm-none-eabi-objcopy -I binary st_ai_output/face_landmark_data.xSPI2.bin --change-addresses 0x70580000 -O ihex face_landmark_data.hex
