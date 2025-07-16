#!/bin/bash

echo "Configurando novo Wi-Fi para a Raspberry Pi"

# Solicita o nome da rede (SSID)
read -p "Digite o nome da rede Wi-Fi (SSID): " ssid

# Solicita a senha da rede
read -sp "Digite a senha do Wi-Fi: " psk
echo ""

# Cria o bloco de configuração
wifi_block="
network={
    ssid=\"$ssid\"
    psk=\"$psk\"
}
"

# Adiciona no arquivo wpa_supplicant.conf
sudo bash -c "echo \"$wifi_block\" >> /etc/wpa_supplicant/wpa_supplicant.conf"

echo "Nova rede adicionada com sucesso!"

# Reinicia a interface de rede
sudo wpa_cli reconfigure

echo "♻️ Rede Wi-Fi atualizada. Verifique com 'hostname -I' se recebeu IP."
