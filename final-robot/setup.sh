#!/bin/bash

echo "Instalando dependencias..."
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip python3-opencv python3-rpi.gpio

echo "Instalando bibliotecas de Python..."
pip3 install --upgrade pip
pip3 install RPi.GPIO opencv-python numpy smbus2

echo "Configurando permisos..."
sudo usermod -aG gpio $(whoami)
sudo usermod -aG i2c $(whoami)

echo "Activando I2C y Camera..."
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_camera 0

echo "Instalación completada. Reinicia para aplicar cambios."
