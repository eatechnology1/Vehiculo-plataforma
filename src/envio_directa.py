"""
Cliente UDP simple para enviar comandos de velocidad lineal y angular a un ESP32.
Este módulo abre un socket UDP y solicita continuamente al usuario dos
valores: velocidad lineal "v" (m/s) y velocidad angular "w" (rad/s). Cada
par se envía como un mensaje ASCII separado por coma ("v,w") a una
dirección IP y puerto del ESP32 preconfigurados.

Constantes:
- ESP_IP (str): Dirección IPv4 objetivo del ESP32.
- ESP_PORT (int): Puerto UDP objetivo en el ESP32.

Comportamiento:
- Ejecuta un bucle infinito leyendo la entrada del usuario con input().
- Construye el mensaje como f"{v},{w}" y lo envía vía UDP usando
    socket.sendto(...).
- Usa el módulo socket de la biblioteca estándar y un socket SOCK_DGRAM.
- No realiza validación ni conversión de entradas; lo que se escriba se envía tal cual.

Uso:
- Ejecutar el módulo en un terminal con acceso de red al ESP32.
- Introducir valores numéricos (por ejemplo, "0.5" o "-1.0") cuando se pida. El programa
    no obliga a que la entrada sea numérica, por lo que entradas no numéricas se enviarán igual.

Limitaciones y sugerencias de mejora:
- Añadir análisis y validación de entrada (por ejemplo, conversión a float) para asegurar que
    el ESP32 reciba comandos numéricos bien formados.
- Implementar manejo de excepciones alrededor de las operaciones de socket y permitir
    un cierre ordenado (cerrar el socket al salir).
- Considerar añadir un timeout de socket, reintentos y confirmaciones a nivel de aplicación
    si se requiere fiabilidad.
- Considerar el uso de argparse para IP/puerto configurables o añadir una interfaz que
    evite el bloqueo por input() en contextos automatizados.
- Tener en cuenta reglas de firewall/red y asegurar que el ESP32 esté escuchando en el puerto UDP configurado.

Dependencias:
- Solo la biblioteca estándar de Python (socket).

Seguridad:
- UDP es sin conexión y no ofrece confidencialidad ni integridad por sí mismo. Evitar enviar datos sensibles
    y considerar autenticación o cifrado si es necesario.
"""
import socket

ESP_IP = "10.226.130.21"  # "192.168.110.26"   # IP de tu ESP32
ESP_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    v = input("v (m/s): ")
    w = input("w (rad/s): ")
    msg = f"{v},{w}"
    sock.sendto(msg.encode(), (ESP_IP, ESP_PORT))
