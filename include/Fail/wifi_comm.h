
/**
 * @class WifiComm
 * @brief Receptor simple de comandos remotos por UDP sobre WiFi para ESP32.
 *
 * Esta clase configura la interfaz WiFi del ESP32 en modo estación, escucha
 * paquetes UDP en un puerto especificado por el usuario, extrae comandos de
 * velocidad lineal y angular de los mensajes entrantes, y registra el tiempo
 * del último comando recibido para soportar la detección de tiempo de espera.
 *
 * Formato de mensaje esperado por parse() y receive():
 *   "v:<velocidad_lineal>,w:<velocidad_angular>"
 * Ejemplo:
 *   "v:0.30,w:-0.50"
 *
 * Notas:
 * - begin() bloquea hasta que el ESP32 se conecta correctamente a la red WiFi.
 *   Los llamantes deben ser conscientes de que es una llamada bloqueante.
 * - receive() lee un único paquete UDP (hasta el tamaño del buffer interno) e
 *   intenta parsear dos valores float. Si el parseo falla, los valores parseados
 *   pueden quedar sin modificar.
 * - timeout() compara el tiempo actual (millis()) con la marca de tiempo del
 *   último comando recibido para determinar si está desactualizado.
 *
 * Patrón de uso:
 *  - Llamar a begin(ssid, password, port) una vez para conectar y empezar UDP.
 *  - Llamar periódicamente a receive(v, w) desde loop() para obtener nuevos comandos.
 *  - Usar timeout(timeoutMs) para detectar pérdida de entrada remota y tomar
 *    acciones de seguridad (por ejemplo, detener el vehículo).
 */

/**
 * @brief Inicializa WiFi en modo estación y comienza a escuchar paquetes UDP.
 *
 * Este método:
 *  - Establece el modo WiFi a WIFI_STA.
 *  - Intenta conectar con el SSID/contraseña proporcionados y bloquea hasta
 *    alcanzar WL_CONNECTED.
 *  - Inicia el listener WiFiUDP interno en el puerto dado.
 *  - Registra el tiempo actual como el del último comando recibido.
 *
 * @param ssid     Cadena C (terminada en NUL) que contiene el SSID de la red WiFi.
 * @param password Cadena C (terminada en NUL) que contiene la contraseña WiFi.
 * @param port     Puerto UDP en el que enlazar para recibir paquetes de comando.
 *
 * @warning Esta función bloquea hasta que se establece la conexión WiFi.
 *          Evitar llamarla desde contextos críticos de tiempo o usar una
 *          estrategia de conexión no bloqueante si se requiere.
 */

/**
 * @brief Intenta recibir y parsear un único paquete de comando UDP.
 *
 * Comprueba si hay un paquete UDP disponible. Si existe, se lee en un buffer
 * interno (dimensionado para evitar desbordes) y se parsea en busca de dos
 * valores float que representan las velocidades lineal (v) y angular (w).
 *
 * @param[out] v Referencia a float que recibirá la velocidad lineal parseada.
 * @param[out] w Referencia a float que recibirá la velocidad angular parseada.
 *
 * @return true  Si se recibió un paquete y se invocó parse() (se actualizó la marca de tiempo).
 * @return false Si no había ningún paquete disponible.
 *
 * @note La función usa sscanf con el patrón exacto "v:%f,w:%f". Si el paquete
 *       entrante no coincide con ese patrón, el parseo puede fallar silenciosamente.
 *       Considerar validar los resultados si se desea mayor robustez.
 */

/**
 * @brief Comprueba si el último comando es más antiguo que un tiempo de espera dado.
 *
 * Compara el tiempo transcurrido (millis() - _lastCmdTime) con el timeout
 * proporcionado en milisegundos.
 *
 * @param timeoutMs Umbral de tiempo de espera en milisegundos.
 * @return true  Si no se ha recibido un comando dentro del tiempo especificado.
 * @return false Si se recibió un comando recientemente (dentro de timeoutMs).
 *
 * @note Utiliza la aritmética con unsigned long de millis() para ser segura ante
 *       el desbordamiento implícito de millis().
 */

/**
 * @brief Parsea un mensaje con el formato "v:<float>,w:<float>".
 *
 * Extrae los valores de velocidad lineal y angular desde una cadena C terminada
 * en NUL. El método espera que el mensaje siga estrictamente el patrón y usa
 * sscanf para el parseo, sin validación adicional.
 *
 * @param msg Cadena de entrada terminada en NUL a parsear.
 * @param[out] v Referencia a float donde se almacenará la velocidad lineal parseada.
 * @param[out] w Referencia a float donde se almacenará la velocidad angular parseada.
 *
 * @warning Al usar sscanf, una entrada malformada puede dejar v y w sin modificar.
 *          Si se requiere mayor robustez, reemplazar o complementar este parser.
 */

/**
 * @brief Socket UDP interno usado para recibir paquetes de comando.
 *
 * Tipo: WiFiUDP
 * Propósito: Enlazar al puerto configurado y recibir datagramas UDP entrantes.
 */

/**
 * @brief Marca de tiempo (millis()) del último comando recibido con éxito.
 *
 * Tipo: unsigned long
 * Propósito: Usada por timeout() para determinar la antigüedad de los comandos entrantes.
 */
#pragma once
#include <WiFi.h>
#include <WiFiUdp.h>

class WifiComm
{
public:
    void begin(const char* ssid, const char* password, uint16_t port)
    {
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);

        while (WiFi.status() != WL_CONNECTED)
        {
            delay(200);
        }

        _udp.begin(port);
        _lastCmdTime = millis();
    }

    bool receive(float &v, float &w)
    {
        int packetSize = _udp.parsePacket();
        if (packetSize)
        {
            char buffer[64];
            int len = _udp.read(buffer, sizeof(buffer) - 1);
            if (len > 0)
            {
                buffer[len] = '\0';
                parse(buffer, v, w);
                _lastCmdTime = millis();
                return true;
            }
        }
        return false;
    }

    bool timeout(uint32_t timeoutMs)
    {
        return (millis() - _lastCmdTime) > timeoutMs;
    }

private:
    WiFiUDP _udp;
    unsigned long _lastCmdTime;

    void parse(const char* msg, float &v, float &w)
    {
        // Espera formato: v:0.30,w:-0.50
        sscanf(msg, "v:%f,w:%f", &v, &w);
    }
};
