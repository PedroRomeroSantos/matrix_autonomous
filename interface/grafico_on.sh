#!/bin/bash
sudo systemctl set-default graphical.target
echo "Modo gráfico ativado, reiniciando em 5 segundos"
sleep 5
sudo reboot
