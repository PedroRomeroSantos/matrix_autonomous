#!/bin/bash
sudo systemctl set-default multi-user.target
echo "Modo gráfico desativado, reiniciando em 5 segundos"
sleep 5
sudo reboot
