#!/bin/bash

echo "Inicializando loopback da câmera CSI para OpenCV..."

# Ativa o loopback
sudo modprobe v4l2loopback devices=1 video_nr=10 card_label="VirtualCam" exclusive_caps=1

echo "Loopback criado em /dev/video10"

# Inicia libcamera-vid como servidor UDP
libcamera-vid -t 0 --width 640 --height 480 --codec yuv420 --inline --listen -o udp://127.0.0.1:8888 &

LIBCAMERA_PID=$!

sleep 2

# Redireciona o stream UDP para o loopback
ffmpeg -i udp://127.0.0.1:8888 -f v4l2 /dev/video10

kill $LIBCAMERA_PID

echo "Encerrado."
