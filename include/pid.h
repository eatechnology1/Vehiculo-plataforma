/**
 * @file pid.h
 * @brief Controlador PID clásico ligero para Arduino/ESP32 (derivada sobre la medición y anti-windup por clamping).
 *
 * Este módulo implementa un PID clásico con:
 *  - Acción proporcional, integral y derivativa.
 *  - Derivada calculada sobre la medición para reducir el efecto del setpoint en la derivada.
 *  - Antiwindup mediante "clamping" de la integral en función de los límites de salida.
 *  - Saturación de salida en un rango configurable.
 *
 * Unidades esperadas:
 *  - setpoint y measurement: rad/s (o cualquier unidad consistente para velocidad).
 *  - dt: segundos.
 *  - salida: normalizada en el intervalo [_outMin, _outMax] (por defecto [-1, 1]).
 *
 * Uso típico:
 *  - Crear instancia con ganancias y límites.
 *  - Llamar a reset() cuando sea necesario (p. ej. al iniciar o cambiar el setpoint de forma brusca).
 *  - Llamar a update(setpoint, measurement, dt) periódicamente para obtener la señal de control.
 *
 * Notas de diseño:
 *  - La derivada se calcula como -(measurement - prevMeasurement) / dt para implementar derivada sobre la medición.
 *  - En la primera llamada a update() la derivada se considera cero.
 *  - El término integral se limita usando _outMin / max(ki, eps) y _outMax / max(ki, eps) para evitar división por cero
 *    y para mantener la acción integral coherente con los límites de salida (ant i-windup por clamping).
 *  - Si dt <= 0, update() devuelve 0.0f para evitar operaciones inválidas.
 *
 * Clase: PID
 * @brief Controlador PID configurable.
 *
 * Constructor:
 * @param kp  Ganancia proporcional.
 * @param ki  Ganancia integral.
 * @param kd  Ganancia derivativa.
 * @param outMin Límite mínimo de salida (por defecto -1.0f).
 * @param outMax Límite máximo de salida (por defecto  1.0f).
 *
 * Método: reset
 * @brief Reinicia el estado interno del controlador.
 * - Pone la integral a 0.
 * - Reinicia la medición previa a 0.
 * - Marca la siguiente ejecución como la primera para omitir la derivada.
 *
 * Método: update
 * @brief Calcula la salida del PID.
 * @param setpoint Referencia deseada (rad/s).
 * @param measurement Valor medido actual (rad/s).
 * @param dt Intervalo de muestreo en segundos (> 0).
 * @return float Señal de control saturada en [_outMin, _outMax] (por defecto [-1, 1]).
 *
 * Comportamiento detallado de update():
 *  - Si dt <= 0 devuelve 0.0f.
 *  - Calcula error = setpoint - measurement.
 *  - Integra error acumulando error * dt.
 *  - Aplica anti-windup por clamping de la integral usando límites derivados de outMin/outMax y ki.
 *  - Calcula la derivada sobre la medición: derivative = -(measurement - prevMeasurement) / dt;
 *    en la primera ejecución la derivada es 0.
 *  - Combina P, I y D: output = kp*error + ki*integral + kd*derivative.
 *  - Aplica saturación final al rango [_outMin, _outMax] antes de devolver el valor.
 *
 * Método: setGains
 * @brief Actualiza dinámicamente las ganancias del controlador.
 * @param kp Nueva ganancia proporcional.
 * @param ki Nueva ganancia integral.
 * @param kd Nueva ganancia derivativa.
 *
 * Variables internas principales:
 *  - _kp, _ki, _kd: ganancias P, I y D.
 *  - _integral: acumulador del término integral.
 *  - _prevMeasurement: última medición usada para la derivada.
 *  - _firstRun: bandera para omitir la derivada en la primera ejecución.
 *  - _outMin, _outMax: límites de salida.
 *
 * Recomendaciones:
 *  - Ajustar las ganancias con técnicas clásicas (Ziegler–Nichols, prueba y error o métodos modernos).
 *  - Si ki es cercano a cero, el anti-windup se protege contra divisiones por cero usando un epsilon interno.
 *  - Asegurar mediciones y dt consistentes (misma unidad y frecuencia de muestreo estable) para un comportamiento robusto.
 */
#pragma once
#include <Arduino.h>

// ======================================================
// ==================== PID CLÁSICO =====================
// ======================================================

class PID
{
public:
    PID(float kp, float ki, float kd, float outMin = -1.0f, float outMax = 1.0f)
        : _kp(kp),
          _ki(ki),
          _kd(kd),
          _outMin(outMin),
          _outMax(outMax)
    {}

    void reset()
    {
        _integral = 0.0f;
        _prevMeasurement = 0.0f;
        _firstRun = true;
    }

    /**
     * @brief Computa la salida del PID
     * 
     * @param setpoint   referencia (rad/s)
     * @param measurement medición actual (rad/s)
     * @param dt         tiempo de muestreo (s)
     * @return float     salida normalizada [-1, 1]
     */
    float update(float setpoint, float measurement, float dt)
    {
        if (dt <= 0.0f)
            return 0.0f;

        float error = setpoint - measurement;

        // ===== Integral =====
        _integral += error * dt;

        // Anti-windup por clamping
        _integral = constrain(_integral, _outMin / max(_ki, 1e-6f),
                                           _outMax / max(_ki, 1e-6f));

        // ===== Derivada sobre la medición =====
        float derivative = 0.0f;
        if (!_firstRun)
        {
            derivative = -(measurement - _prevMeasurement) / dt;
        }

        _prevMeasurement = measurement;
        _firstRun = false;

        // ===== PID =====
        float output =
            _kp * error +
            _ki * _integral +
            _kd * derivative;

        // Saturación final
        return constrain(output, _outMin, _outMax);
    }

    // ===== Setters dinámicos =====
    void setGains(float kp, float ki, float kd)
    {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }

private:
    float _kp;
    float _ki;
    float _kd;

    float _integral = 0.0f;
    float _prevMeasurement = 0.0f;
    bool  _firstRun = true;

    float _outMin;
    float _outMax;
};
