"""
Cliente UDP simple para enviar un objetivo (x, y) y una velocidad lineal v a un ESP32.

Formato del mensaje:
    "x,y,v"

donde:
    x: posición objetivo en X [m]
    y: posición objetivo en Y [m]
    v: velocidad lineal deseada [m/s]
"""

import socket

ESP_IP = "192.168.110.26"   # IP de tu ESP32
ESP_PORT = 8888             # Debe coincidir con UDP_PORT en config.h

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    x = input("x objetivo (m): ")
    y = input("y objetivo (m): ")
    v = input("v lineal (m/s): ")
    msg = f"{x},{y},{v}"
    sock.sendto(msg.encode(), (ESP_IP, ESP_PORT))
