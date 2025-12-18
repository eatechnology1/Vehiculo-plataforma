
/*
 * config.h — Configuración y asignación de pines para la plataforma robótica ESP32-S3
 *
 * Propósito
 *  - Cabecera de configuración centralizada que mapea pines de hardware, canales PWM,
 *    entradas de encoders, geometría del robot, límites de movimiento y parámetros de red.
 *  - Diseñada para ser incluida por módulos de firmware que controlan motores, leen
 *    encoders, realizan control de movimiento y gestionan comandos de red.
 *
 * Secciones y símbolos (breve)
 * 1) TB6612FNG (driver de motor)
 *    - PWM_L, IN1_L, IN2_L   : Pines que controlan el motor izquierdo (PWM + dirección)
 *    - PWM_R, IN1_R, IN2_R   : Pines que controlan el motor derecho (PWM + dirección)
 *    - PWM_P, IN1_P, IN2_P   : Pines que controlan el motor de la plataforma/elevador (PWM + dirección)
 *    - STBY                  : Pin de standby/enable del driver TB6612FNG
 *    - Observación: STBY normalmente debe ponerse en alto para habilitar las salidas; usarlo para
 *      deshabilitar rápidamente todos los drivers en condiciones de fallo o seguridad.
 *
 * 2) Encoders
 *    - ENC_L1_A / ENC_L1_B y ENC_L2_A / ENC_L2_B:
 *        Entradas de encoder en cuadratura para los dos motores de la oruga izquierda.
 *    - ENC_R1_A / ENC_R1_B y ENC_R2_A / ENC_R2_B:
 *        Entradas de encoder en cuadratura para los dos motores de la oruga derecha.
 *    - ENC_P_A / ENC_P_B:
 *        Entradas de encoder en cuadratura para la plataforma/elevador.
 *    - Observaciones:
 *      - Estos son números de GPIO; asegúrese de que los pines seleccionados soporten interrupciones externas
 *        o el método de captura de encoder elegido en ESP32-S3.
 *      - Considere filtrado o antirrebote y use manejadores seguros para interrupciones o periféricos
 *        dedicados (PCNT/RMT) para altas tasas de pulsos.
 *
 * 3) Configuración PWM (LEDC)
 *    - PWM_CH_L, PWM_CH_R, PWM_CH_P: Canales LEDC asignados a cada salida PWM de motor.
 *    - PWM_FREQ: Frecuencia PWM en Hz usada para el control de motores (ej. 20 kHz).
 *    - PWM_RES: Resolución PWM en bits (ej. 8).
 *    - PWM_MAX: Deber máximo PWM basado en la resolución (ej. 2^RES - 1).
 *    - Observaciones:
 *      - Asegúrese de que las asignaciones de canal/timer no colisionen y sean compatibles con
 *        los GPIOs y timers seleccionados en ESP32-S3.
 *
 * 4) Geometría y cinemática del robot
 *    - TRACK_WIDTH : Distancia entre los centros de tracción izquierdo y derecho (metros).
 *    - WHEEL_RADIUS: Radio de las ruedas motrices (metros).
 *    - Uso: Convertir velocidades lineales/angulares del chasis a velocidades angulares de rueda
 *      y viceversa al implementar control diferencial.
 *
 * 5) Constantes de encoders/ángulos
 *    - PPR       : Pulsos por revolución de los encoders (resolución del encoder).
 *    - TWO_PI_F  : Constante 2 * pi para conversiones en radianes.
 *    - Observaciones:
 *      - Use PPR junto con relaciones de reducción (si existen) para calcular rotaciones de rueda y
 *        distancia lineal por pulso de encoder.
 *
 * 6) Límites de movimiento
 *    - V_MAX: Velocidad lineal máxima permitida (m/s).
 *    - W_MAX: Velocidad angular máxima permitida (rad/s).
 *    - Observaciones:
 *      - Aplique estos límites en los controladores de movimiento para evitar comandos inseguros.
 *
 * 7) Wi‑Fi y redes
 *    - WIFI_SSID, WIFI_PASSWORD: Credenciales Wi‑Fi usadas para conectar el dispositivo.
 *      No incluir credenciales en firmware para producción; usar almacenamiento seguro.
 *    - UDP_PORT: Puerto UDP usado para recibir comandos remotos.
 *    - CMD_TIMEOUT_MS: Timeout de comandos en milisegundos; usado para detectar comandos obsoletos.
 *
 * Buenas prácticas, precauciones y notas
 *  - Selección de pines: Verifique el comportamiento de arranque de cada GPIO en ESP32-S3 para evitar
 *    fallos de boot (algunos pines no deben ser forzados a ciertos niveles al arrancar).
 *  - Seguridad: Use STBY y watchdogs de software para detener motores ante fallos o pérdida de comunicación.
 *  - Seguridad de credenciales: Evite almacenar contraseñas en texto plano en control de versiones; prefiera
 *    configuración en tiempo de ejecución o almacenamiento encriptado.
 *  - Mantenibilidad: Considere agrupar IDs de motores/encoders en enums o structs
 *    y proporcionar funciones de acceso para reducir el uso directo de macros.
 *  - Calibración: TRACK_WIDTH, WHEEL_RADIUS y PPR deben calibrarse en el robot ensamblado
 *    para obtener odometría precisa.
 *
 * Ejemplos de uso
 *  - Para calcular la velocidad angular de la rueda (rad/s) a partir de la velocidad lineal v (m/s):
 *      omega = v / WHEEL_RADIUS
 *  - Para calcular ticks de encoder por revolución de rueda:
 *      ticks_per_rev = PPR  (aplicar relación de engranajes si corresponde)
 *
 * Revisión / autoría
 *  - Archivo: include/config.h
 *  - Propósito: Configuración de hardware y tiempo de ejecución para la plataforma del vehículo.
 */
#pragma once

// ======================================================
// ================== TB6612FNG ==========================
// ======================================================

// ---- Motor Izquierdo (plataforma izquierda) ----
#define PWM_L      9     // pwmIZQUIERDO
#define IN1_L      11     // in1IZQUIERDO
#define IN2_L      10     // in2IZQUIERDO

// ---- Motor Derecho (plataforma derecha) ----
#define PWM_R     12     // pwmderecho
#define IN1_R     14     // in1derecho
#define IN2_R     13     // in2derecho

// ---- Motor Izquierdo (plataforma izquierda) ----
#define PWM_P     18     // pwmplataforma
#define IN1_P      8     // in1plataforma
#define IN2_P      3     // in2plataforma

// ---- Standby driver ----
#define STBY       46     // standby

// ======================================================
// ================== ENCODERS ===========================
// ======================================================

// ---- Oruga Izquierda (2 motores) ----
#define ENC_L1_A   4
#define ENC_L1_B   1

#define ENC_L2_A   41
#define ENC_L2_B   40

// ---- Oruga Derecha (2 motores) ----
#define ENC_R1_A   2
#define ENC_R1_B   42

#define ENC_R2_A   39
#define ENC_R2_B   38

// ---- Elevador Plataforma ----
#define ENC_P_A    37
#define ENC_P_B    36



// ======================================================
// ================== PWM (LEDC) =========================
// ======================================================

#define PWM_CH_L   0
#define PWM_CH_R   1
#define PWM_CH_P   2   // Elevador plataforma

#define PWM_FREQ  20000
#define PWM_RES   8
#define PWM_MAX   255


// ======================================================
// ================== ROBOT ==============================
// ======================================================

// Distancia entre los centros de tracción (oruga izq/dcha)
#define TRACK_WIDTH   0.135f   // [m]

// Radio efectivo de la rueda/oruga
#define WHEEL_RADIUS  0.030f   // [m]

// ======================================================
// ================== ENCODERS ===========================
// ======================================================

#define PPR       4200
#define TWO_PI_F    6.283185307f

// ======================================================
// ================== LIMITES ============================
// ======================================================

#define V_MAX     0.6f
#define W_MAX     2.0f

// ======================================================
// ================== WIFI ===============================
// ======================================================

#define WIFI_SSID     "Calderon Ramon" //"TU_RED_WIFI"
#define WIFI_PASSWORD "Lina.Alex.321" //"TU_PASSWORD"
#define UDP_PORT      8888
#define CMD_TIMEOUT_MS  5000
