
/**
 * @file motor_driver.h
 * @brief Clases de control de motores para una plataforma de vehículo basada en ESP32.
 *
 * Este encabezado proporciona una abstracción ligera Motor que controla
 * un motor DC bidireccional usando un puente H (dos pines de dirección
 * y un pin PWM), y un MotorManager que agrupa tres instancias Motor
 * (oruga izquierda, oruga derecha y plataforma/elevador).
 *
 * La implementación depende de constantes definidas en "config.h" tales como:
 *  - PWM_FREQ  : frecuencia PWM usada por ledc
 *  - PWM_RES   : resolución PWM para ledcSetup
 *  - PWM_MAX   : duty máximo correspondiente a escala completa
 *  - PWM_L/R/P : números de pin PWM para motores izquierdo/derecho/plataforma
 *  - IN1_L/... : números de pin de control de dirección
 *  - PWM_CH_L/... : canales PWM asignados a cada motor
 *  - STBY      : pin de espera/enable para el controlador de motor
 *
 * La clase Motor utiliza la API ledc del ESP32 (ledcSetup/ledcAttachPin/ledcWrite)
 * para generar PWM y dos salidas digitales para establecer la dirección del motor.
 * Los valores pasados a los métodos de control se normalizan al rango [-1.0, 1.0].
 *
 * Seguridad de hilos: las clases están diseñadas para un uso simple de un solo hilo
 * al estilo Arduino. Si se usan desde múltiples tareas/ISRs, se recomienda
 * sincronización externa.
 */

/**
 * @class Motor
 * @brief Abstracción para un único motor DC bidireccional controlado por un puente H.
 *
 * Responsabilidades:
 *  - Configurar el canal PWM y los pines de dirección en begin()
 *  - Convertir una señal de control normalizada en dirección y duty PWM
 *  - Proveer un helper stop() que ponga el motor en un estado seguro inactivo
 *
 * Modelo de control:
 *  - El método set(float u) acepta u en el intervalo cerrado [-1.0, 1.0].
 *    - u > 0 : dirección hacia adelante (in1 HIGH, in2 LOW) con duty = u * PWM_MAX
 *    - u < 0 : dirección hacia atrás (in1 LOW, in2 HIGH) con duty = -u * PWM_MAX
 *    - u == 0: ambos pines de dirección LOW y duty 0 (rueda libre/inactivo)
 *
 * Notas de hardware:
 *  - PWM se implementa mediante ledc en el ESP32; pwmChannel y pwmPin
 *    proporcionados al constructor deben ser válidos para ledcAttachPin.
 *  - Los pines de dirección se configuran como salidas digitales.
 *
 * Parámetros del constructor:
 *  - pwmPin     : pin GPIO usado para salida PWM
 *  - in1Pin     : pin GPIO que controla la entrada 1 del puente H
 *  - in2Pin     : pin GPIO que controla la entrada 2 del puente H
 *  - pwmChannel : índice de canal PWM ledc a usar para este motor
 */
 
/**
 * @brief Inicializar GPIOs del motor y canal PWM.
 *
 * Llama a:
 *  - pinMode para pines de dirección
 *  - ledcSetup(pwmChannel, PWM_FREQ, PWM_RES)
 *  - ledcAttachPin(pwmPin, pwmChannel)
 *  - stop() para asegurar que el motor quede inactivo tras la inicialización
 */

/**
 * @brief Establecer el comando del motor con un valor normalizado.
 * @param u Señal de control normalizada en [-1.0, 1.0].
 *
 * Comportamiento:
 *  - Restringe u a [-1.0, 1.0].
 *  - Selecciona los pines de dirección según el signo de u.
 *  - Calcula duty = round(|u| * PWM_MAX) y lo escribe en el canal PWM.
 *  - Utiliza digitalWrite para los pines de dirección y ledcWrite para el duty.
 *
 * Notas:
 *  - Un valor exactamente 0 desactiva ambos pines de dirección y pone duty a 0.
 *  - El tipo de duty y la escala usan PWM_MAX definido en config.h.
 */

/**
 * @brief Detener inmediatamente el motor y colocar las salidas en un estado seguro.
 *
 * Acciones:
 *  - Establece el duty interno a 0
 *  - Lleva ambos pines de dirección a LOW (dependiendo del driver: puede ser rueda libre o frenado)
 *  - Escribe 0 en el canal PWM
 */

/**
 * @class MotorManager
 * @brief Gestor de alto nivel para los tres motores usados por la plataforma del vehículo.
 *
 * Agrega:
 *  - motorLeft     : motor de tracción izquierdo
 *  - motorRight    : motor de tracción derecho
 *  - motorPlatform : motor del elevador/plataforma
 *
 * Las asignaciones concretas de pines y canales se proveen en config.h mediante
 * constantes en tiempo de compilación. MotorManager se encarga de la configuración
 * global (incluyendo afirmar el pin STBY/enable) e interfaces simples por motor.
 */

/**
 * @brief Inicializar el sistema de control de motores.
 *
 * Comportamiento:
 *  - Configura el pin STBY (standby/enable) como OUTPUT y lo pone en HIGH
 *    para habilitar la placa controladora de motores.
 *  - Llama a begin() en cada instancia Motor para configurar pines y canales PWM.
 *
 * Nota de hardware:
 *  - Si el controlador de motores requiere que STBY sea toggled por seguridad o reset,
 *    ajuste el uso aquí según corresponda.
 */

/**
 * @brief Establecer el comando del motor de la oruga izquierda.
 * @param u Señal de control normalizada en [-1.0, 1.0] reenviada al Motor izquierdo.
 */

/**
 * @brief Establecer el comando del motor de la oruga derecha.
 * @param u Señal de control normalizada en [-1.0, 1.0] reenviada al Motor derecho.
 */

/**
 * @brief Establecer el comando del motor de la plataforma/elevador.
 * @param u Señal de control normalizada en [-1.0, 1.0] reenviada al Motor de la plataforma.
 */

/**
 * @brief Detener todos los motores inmediatamente.
 *
 * Comportamiento:
 *  - Llama a stop() en motorLeft, motorRight y motorPlatform.
 *  - Deja el estado del pin STBY sin cambios (habilitado). Si se requiere una
 *    desactivación completa del hardware, ponga STBY en LOW externamente o
 *    extienda este método.
 */
#pragma once
#include <Arduino.h>
#include "config.h"

// ======================================================
// ================== CLASE MOTOR =======================
// ======================================================

class Motor {
public:
    Motor(uint8_t pwmPin,
          uint8_t in1Pin,
          uint8_t in2Pin,
          uint8_t pwmChannel)
        : pwmPin(pwmPin),
          in1Pin(in1Pin),
          in2Pin(in2Pin),
          pwmChannel(pwmChannel),
          duty(0) {}

    void begin() {
        pinMode(in1Pin, OUTPUT);
        pinMode(in2Pin, OUTPUT);

        ledcSetup(pwmChannel, PWM_FREQ, PWM_RES);
        ledcAttachPin(pwmPin, pwmChannel);

        stop();
    }

    // --------------------------------------------------
    // u ∈ [-1, 1]
    // --------------------------------------------------
    void set(float u) {
        u = constrain(u, -1.0f, 1.0f);

        if (u > 0.0f) {
            digitalWrite(in1Pin, HIGH);
            digitalWrite(in2Pin, LOW);
            duty = static_cast<uint32_t>(u * PWM_MAX);
        }
        else if (u < 0.0f) {
            digitalWrite(in1Pin, LOW);
            digitalWrite(in2Pin, HIGH);
            duty = static_cast<uint32_t>(-u * PWM_MAX);
        }
        else {
            duty = 0;
            digitalWrite(in1Pin, LOW);
            digitalWrite(in2Pin, LOW);
        }

        ledcWrite(pwmChannel, duty);
    }

    void stop() {
        duty = 0;
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
        ledcWrite(pwmChannel, 0);
    }

private:
    uint8_t pwmPin;
    uint8_t in1Pin;
    uint8_t in2Pin;
    uint8_t pwmChannel;

    uint32_t duty;
};

// ======================================================
// =============== GESTOR DE MOTORES ====================
// ======================================================

class MotorManager {
public:
    MotorManager()
        : motorLeft (PWM_L, IN1_L, IN2_L, PWM_CH_L),
          motorRight(PWM_R, IN1_R, IN2_R, PWM_CH_R),
          motorPlatform(PWM_P, IN1_P, IN2_P, PWM_CH_P) {}

    void begin() {
        pinMode(STBY, OUTPUT);
        digitalWrite(STBY, HIGH);

        motorLeft.begin();
        motorRight.begin();
        motorPlatform.begin();
    }

    // ---------------- Orugas ---------------------------
    void setLeft(float u) {
        motorLeft.set(u);
    }

    void setRight(float u) {
        motorRight.set(u);
    }

    // ---------------- Elevador -------------------------
    void setPlatform(float u) {
        motorPlatform.set(u);
    }

    // ---------------- Seguridad ------------------------
    void stopAll() {
        motorLeft.stop();
        motorRight.stop();
        motorPlatform.stop();
    }

private:
    Motor motorLeft;
    Motor motorRight;
    Motor motorPlatform;
};
