
/**
 * @file encoder.h
 * @brief Interfaz para encoders cuádruples y gestor para plataforma robótica basada en ESP32.
 *
 * Resumen
 * -------
 * Este encabezado define:
 *  - Encoder: una clase ligera que asocia interrupciones a dos pines GPIO y
 *    mantiene un contador de impulsos (ticks) y una velocidad angular estimada (omega).
 *  - EncoderManager: un agregador para cinco encoders utilizados en la plataforma
 *    (dos izquierdos, dos derechos y uno auxiliar/plataforma).
 *
 * Objetivo y alcance
 * ------------------
 * El objetivo es proporcionar una abstracción sencilla y eficiente para la
 * lectura de encoders cuádruples en microcontroladores ESP32/ESP32-S3, permitiendo
 * la obtención periódica de la velocidad angular en radianes por segundo y un
 * acceso directo al número de ticks acumulados. La implementación pretende ser
 * adecuada para controladores de bajo coste y tareas de control en lazo cerrado,
 * aunque no sustituye mecanismos hardware (PCNT) cuando se requiera alta fiabilidad
 * a altas frecuencias.
 *
 * Diseño y comportamiento
 * -----------------------
 * - Construcción:
 *     Los objetos Encoder se crean indicando dos pines GPIO (pinA, pinB) y la
 *     resolución del encoder en pulsos por revolución (ppr).
 *
 * - begin():
 *     * Configura pinA y pinB como INPUT_PULLUP.
 *     * Adjunta dos manejadores de interrupción (uno por canal) mediante attachInterruptArg.
 *     * Inicializa la marca temporal para el cálculo de velocidad.
 *
 * - ISRs (isrA, isrB):
 *     * Las ISRs reciben un puntero void * al objeto Encoder suministrado por attachInterruptArg.
 *     * Cada ISR lee ambos canales y actualiza el contador de ticks usando la lógica de cuadratura:
 *         - isrA: ticks += (A == B) ? +1 : -1
 *         - isrB: ticks += (A != B) ? +1 : -1
 *     * El miembro ticks está declarado volatile long para permitir su modificación segura desde la ISR.
 *     * NOTA: digitalRead() y otras llamadas del API Arduino pueden no ser seguras dentro de ISRs que
 *       se ejecutan en IRAM en todas las variantes de ESP32. En caso de dudas, mover/anotar la ISR
 *       con IRAM_ATTR y evitar llamadas lentas.
 *
 * - update():
 *     * Calcula el intervalo transcurrido desde la última actualización usando micros() y obtiene
 *       la velocidad angular (omega) en rad/s mediante:
 *         omega = (delta_ticks * 2*pi) / (ppr * dt)
 *     * delta_ticks = ticks - lastTicks
 *     * dt está expresado en segundos (conversión desde micros()).
 *     * update() debe invocarse periódicamente desde el bucle principal o desde una tarea.
 *
 * - getOmega(), getTicks(), reset():
 *     * Accesores para la velocidad angular calculada y el contador de ticks.
 *     * reset() pone a cero ticks y omega y reinicia la marca temporal de actualización.
 *
 * Unidades y semántica
 * --------------------
 * - ticks: cuenta entera de impulsos (incremento/decremento por cada flanco detectado según la lógica de ISR). Tipo: long.
 * - ppr: pulsos por revolución utilizados para convertir ticks en revoluciones. La fórmula en update()
 *   asume que ppr corresponde a la convención de ticks por revolución empleada (tener en cuenta multiplexado/
 *   cuadratura).
 * - omega: radianes por segundo (float). Se usa TWO_PI_F como constante para 2*pi.
 *
 * Concurrencia y consideraciones de seguridad
 * -------------------------------------------
 * - ticks es volatile y se actualiza dentro de ISRs. update() lee ticks sin deshabilitar interrupciones.
 *   Esto es aceptable en muchos casos, pero puede producir discrepancias de ±1 tick si se produce una
 *   interrupción entre la lectura y la actualización de lastTicks. Si se requiere atomicidad estricta,
 *   considerar deshabilitar temporalmente interrupciones alrededor de la lectura o utilizar primitivas atómicas.
 * - Las ISRs emplean digitalRead(), que puede ser lenta o no segura en ISR en algunas placas. Para encoders
 *   de alta frecuencia o requisitos real-time estrictos se recomienda:
 *     * Usar el periférico PCNT del ESP32.
 *     * Colocar ISRs en IRAM (IRAM_ATTR) y emplear lecturas directas de registros GPIO.
 *
 * Gestor de encoders (EncoderManager)
 * -----------------------------------
 * - Agrega cinco instancias Encoder:
 *     encL1, encL2: encoders de la rueda izquierda
 *     encR1, encR2: encoders de la rueda derecha
 *     encP        : encoder auxiliar/plataforma
 * - begin(): llama a begin() de cada encoder contenido.
 * - update(): actualiza todos los encoders; debe llamarse periódicamente.
 * - omegaLeft(): devuelve la velocidad angular media de los dos encoders izquierdos,
 *   multiplicada por -0.5 (la inversión de signo corresponde a la convención geométrica del vehículo).
 * - omegaRight(): devuelve la velocidad angular media de los dos encoders derechos, multiplicada por +0.5.
 * - omegaPlatform(): devuelve el omega del encoder auxiliar encP.
 * - resetAll(): reinicia ticks y omega en todos los encoders.
 *
 * Configuración y dependencias
 * ----------------------------
 * - Este fichero depende de:
 *     - Arduino.h para pinMode, digitalRead, attachInterruptArg y micros().
 *     - config.h para definiciones de pines y PPR:
 *         ENC_L1_A, ENC_L1_B, ENC_L2_A, ENC_L2_B, ENC_R1_A, ENC_R1_B,
 *         ENC_R2_A, ENC_R2_B, ENC_P_A, ENC_P_B y PPR.
 *     - TWO_PI_F (2*pi) debe estar disponible; en caso contrario, definir la constante adecuada.
 *
 * Ejemplo de uso (resumen)
 * ------------------------
 * - En setup():
 *     encoderManager.begin();
 * - En loop() o en una tarea periódica:
 *     encoderManager.update();
 *     float izquierda = encoderManager.omegaLeft();   // rad/s
 *     float derecha  = encoderManager.omegaRight();  // rad/s
 *
 * Limitaciones y recomendaciones para sustentación de tesis
 * ---------------------------------------------------------
 * - Para estimaciones de velocidad más robustas, considerar filtrar omega (p. ej. promedio móvil o filtro de Kalman)
 *   o calcular la velocidad sobre una ventana de ticks acumulados para reducir el ruido de cuantización a bajas velocidades.
 * - Validar experimentalmente la correspondencia entre PPR, la numeración de ticks y las relaciones de transmisión mecánica.
 * - Documentar la seguridad de las ISRs en la placa objetivo: si la implementación requiere alta frecuencia de interrupciones,
 *   justificar la elección de usar ISRs con digitalRead() frente a soluciones basadas en hardware (PCNT) y reportar medidas
 *   de jitter y error de conteo en el anexo experimental.
 *
 * Notas finales
 * -------------
 * Este documento pretende servir como descripción técnica y guía de uso para la implementación de encoders
 * en el trabajo de grado. En la sustentación se recomienda presentar resultados experimentales que respalden
 * la elección de parámetros (PPR, ventana de muestreo) y las posibles mejoras propuestas.
 */
#pragma once
#include <Arduino.h>
#include "config.h"

// ======================================================
// ================= CLASE ENCODER ======================
// ======================================================

class Encoder
{
public:
    Encoder(uint8_t pinA, uint8_t pinB, int ppr)
        : pinA(pinA), pinB(pinB), ppr(ppr),
          ticks(0), lastTicks(0),
          omega(0.0f), lastTime(0) {}

    void begin()
    {
        pinMode(pinA, INPUT_PULLUP);
        pinMode(pinB, INPUT_PULLUP);

        attachInterruptArg(pinA, isrA, this, CHANGE);
        attachInterruptArg(pinB, isrB, this, CHANGE);

        lastTime = micros();
    }

    void update()
    {
        uint32_t now = micros();
        float dt = (now - lastTime) * 1e-6f;
        if (dt <= 0.0f)
            return;

        long delta = ticks - lastTicks;
        lastTicks = ticks;
        lastTime = now;

        omega = (delta * TWO_PI_F) / (ppr * dt);
    }

    float getOmega() const
    {
        return omega;
    }

    long getTicks() const
    {
        return ticks;
    }

    void reset()
    {
        ticks = 0;
        lastTicks = 0;
        omega = 0.0f;
        lastTime = micros();
    }

private:
    uint8_t pinA, pinB;
    int ppr;

    volatile long ticks;
    long lastTicks;

    float omega;
    uint32_t lastTime;

    // ---------- ISR SIN IRAM_ATTR ----------
    static void isrA(void *arg)
    {
        Encoder *enc = static_cast<Encoder *>(arg);
        bool A = digitalRead(enc->pinA);
        bool B = digitalRead(enc->pinB);
        enc->ticks += (A == B) ? 1 : -1;
    }

    static void isrB(void *arg)
    {
        Encoder *enc = static_cast<Encoder *>(arg);
        bool A = digitalRead(enc->pinA);
        bool B = digitalRead(enc->pinB);
        enc->ticks += (A != B) ? 1 : -1;
    }
};

// ======================================================
// ============== GESTOR DE ENCODERS ====================
// ======================================================

class EncoderManager
{
public:
    EncoderManager()
        : encL1(ENC_L1_A, ENC_L1_B, PPR),
          encL2(ENC_L2_A, ENC_L2_B, PPR),
          encR1(ENC_R1_A, ENC_R1_B, PPR),
          encR2(ENC_R2_A, ENC_R2_B, PPR),
          encP(ENC_P_A, ENC_P_B, PPR) {}

    void begin()
    {
        encL1.begin();
        encL2.begin();
        encR1.begin();
        encR2.begin();
        encP.begin();
    }

    void update()
    {
        encL1.update();
        encL2.update();
        encR1.update();
        encR2.update();
        encP.update();
    }

    float omegaLeft() const
    {
        return -0.5f * (encL1.getOmega() + encL2.getOmega());
    }

    float omegaRight() const
    {
        return 0.5f * (encR1.getOmega() + encR2.getOmega());
    }

    float omegaPlatform() const
    {
        return encP.getOmega();
    }

    void resetAll()
    {
        encL1.reset();
        encL2.reset();
        encR1.reset();
        encR2.reset();
        encP.reset();
    }

private:
    Encoder encL1, encL2;
    Encoder encR1, encR2;
    Encoder encP;
};
