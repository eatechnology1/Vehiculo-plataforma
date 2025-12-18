
/**
 * @file kinematics.h
 * @brief Estimador de cinemática diferencial para integración de la pose.
 *
 * @descripcion
 * Esta unidad implementa un estimador simple de cinemática diferencial
 * orientado a plataformas móviles con dos ruedas motrices (differential-drive).
 * Su propósito es integrar la pose planar del robot (x, y, theta) y estimar
 * las velocidades del chasis (lineal v y angular w) a partir de las velocidades
 * angulares medidas de las ruedas izquierda y derecha.
 *
 * Convenciones y sistema de referencia
 * - Coordenadas: se usa un marco inercial plano (x, y) con theta medido en
 *   radianes respecto al eje +x (convención estándar de brújula matemática,
 *   sentido positivo antihorario).
 * - Velocidades angulares de rueda (omegaL, omegaR): rad/s.
 * - Radio de rueda (WHEEL_RADIUS) y separación de ejes (TRACK_WIDTH) se
 *   suministran desde config.h y deben estar en unidades SI (metros).
 * - v (velocidad lineal del chasis): m/s. w (velocidad angular del chasis): rad/s.
 *
 * Modelo y ecuaciones
 * - vL = omegaL * WHEEL_RADIUS        // velocidad lineal rueda izquierda (m/s)
 * - vR = omegaR * WHEEL_RADIUS        // velocidad lineal rueda derecha  (m/s)
 * - v  = 0.5 * (vR + vL)              // velocidad lineal del centro del chasis
 * - w  = (vR - vL) / TRACK_WIDTH      // velocidad angular alrededor del centro
 *
 * Integración temporal (Forward Euler)
 * - Si dt es el intervalo de tiempo (s) entre muestras:
 *   x     := x + v * cos(theta) * dt
 *   y     := y + v * sin(theta) * dt
 *   theta := theta + w * dt
 *
 * Normalización angular
 * - Después de integrar theta, se normaliza al rango [-PI, PI] usando las
 *   constantes PI y TWO_PI_F provistas en config.h.
 *
 * Implementación temporal y precisión
 * - La clase usa micros() (Arduino) para obtener la marca de tiempo en
 *   microsegundos. lastTime se almacena como uint32_t. El cálculo de dt se
 *   hace como (now - lastTime) * 1e-6f para obtener segundos.
 * - Nota sobre overflow de micros(): micros() es un contador de 32 bits y
 *   envuelve aproximadamente cada 71.6 minutos. La resta entre valores unsigned
 *   produce la diferencia modular correcta siempre que la diferencia real sea
 *   menor que el periodo de envoltura. Si el tiempo entre llamadas a update()
 *   excede un umbral razonable (p. ej., varias décimas de segundo) es
 *   recomendable detectar y manejar el caso (por ejemplo, ignorar dt excesivo).
 *
 * Suposiciones y limitaciones
 * - No se modelan deslizamientos (skid), holguras mecánicas ni errores de
 *   calibración de radio/track. Estos efectos introducen deriva en la odometría.
 * - La integración por Euler es de primer orden; para dt grandes o dinámicas
 *   rápidas, el error de integración puede ser significativo. Considerar
 *   métodos de orden superior (Runge-Kutta) si se requiere mayor precisión.
 * - El estimador calcula la pose únicamente a partir de velocidades angulares
 *   instantáneas: requiere que omegaL y omegaR sean mediciones fiables y con
 *   ruido razonablemente bajo.
 *
 * Robustez numérica y recomendaciones prácticas
 * - Mantener una frecuencia de muestreo constante y con dt pequeño (ej. >50-100 Hz
 *   si se desea buena resolución de trayectoria) reduce errores de integración.
 * - Validar y calibrar WHEEL_RADIUS y TRACK_WIDTH con pruebas controladas
 *   (recorridos rectos conocidos, giros de 360º) para minimizar sesgos.
 * - Si se dispone de sensores adicionales (IMU, GPS, cámaras) usar fusiones
 *   (EKF, UKF, fusión de sensores) para corregir deriva de la odometría.
 * - Para estimación de incertidumbre, propagar covarianzas asociadas a las
 *   mediciones de velocidad angular y modelar ruido de proceso en un filtro.
 *
 * Casos de uso y API
 * - Construcción: Kinematics() inicializa pose y velocidades a cero.
 * - begin(): inicializa la base temporal (lastTime = micros()) antes de usar.
 * - reset(): pone pose y velocidades a cero y reinicia la referencia temporal.
 * - update(omegaL, omegaR): integrar la pose usando las velocidades angulares
 *   medidas; si dt <= 0 la llamada se ignora.
 * - getX(), getY(), getTheta(), getV(), getW(): acceder al estado estimado.
 *
 * Validación experimental sugerida (capítulos para tesis)
 * - Prueba 1 (recta): mandar velocidades iguales a ambas ruedas, medir la
 *   desviación lateral después de N metros. Permite estimar error en WHEEL_RADIUS.
 * - Prueba 2 (rotación en sitio): mandar velocidades opuestas, medir ángulo
 *   alcanzado tras T segundos. Permite estimar error en TRACK_WIDTH y diferencias
 *   en fricción.
 * - Prueba 3 (perfil de velocidad): aplicar perfil conocido (rampa/onda) y
 *   comparar trayectoria medida con sistemas externos de referencia (Vicon,
 *   cámara motion-capture o GPS en exteriores).
 * - Análisis de errores: calcular RMSE en posición y orientación, estudiar
 *   dependencia con dt, ruido de medición y desgaste de ruedas.
 *
 * Posibles extensiones (direcciones de investigación / mejoras)
 * - Sustituir integración Euler por RK4 para reducir error a igual dt.
 * - Introducir modelado de deslizamiento en eje longitudinal/lateral.
 * - Fusionar odometría con IMU/GNSS mediante filtros Bayesianos para obtener
 *   estimaciones más robustas y covarianzas en tiempo real.
 * - Estimar parámetros del modelo (WHEEL_RADIUS, TRACK_WIDTH) en línea mediante
 *   técnicas de identificación adaptativa o SLAM.
 *
 * Referencias y bibliografía recomendada
 * - Siegwart, Nourbakhsh, Scaramuzza. "Introduction to Autonomous Mobile Robots".
 * - Thrun, Burgard, Fox. "Probabilistic Robotics" (capítulos sobre odometría y fusión).
 *
 * Notas finales
 * - La implementación y las constantes relacionadas (WHEEL_RADIUS, TRACK_WIDTH,
 *   PI, TWO_PI_F) se encuentran en config.h. Revisar y documentar esas constantes
 *   con sus incertidumbres experimentales en la sección de métodos de la tesis.
 *
 * Autor: Implementación de cinemática diferencial (comentario en español para tesis)
 * Fecha: (indicar fecha de inclusión en el repositorio)
 */
#pragma once
#include <Arduino.h>
#include "config.h"

// ======================================================
// ================= CINEMATICA DIFERENCIAL =============
// ======================================================

class Kinematics {
public:
    Kinematics()
        : x(0.0f), y(0.0f), theta(0.0f),
          v(0.0f), w(0.0f),
          lastTime(0) {}

    void begin() {
        lastTime = micros();
    }

    void reset() {
        x = 0.0f;
        y = 0.0f;
        theta = 0.0f;
        v = 0.0f;
        w = 0.0f;
        lastTime = micros();
    }

    // ==================================================
    // Actualización usando velocidades angulares medidas
    // ==================================================
    void update(float omegaL, float omegaR) {
        uint32_t now = micros();
        float dt = (now - lastTime) * 1e-6f;
        if (dt <= 0.0f) return;
        lastTime = now;

        // Velocidades lineales de ruedas
        float vL = omegaL * WHEEL_RADIUS;
        float vR = omegaR * WHEEL_RADIUS;

        // Velocidades del chasis
        v = 0.5f * (vR + vL);
        w = (vR - vL) / TRACK_WIDTH;

        // Integración (Euler)
        x     += v * cosf(theta) * dt;
        y     += v * sinf(theta) * dt;
        theta += w * dt;

        normalizeAngle();
    }

    // ================= Getters ========================
    float getX() const { return x; }
    float getY() const { return y; }
    float getTheta() const { return theta; }
    float getV() const { return v; }
    float getW() const { return w; }

private:
    float x, y, theta;   // Pose
    float v, w;          // Velocidades del chasis
    uint32_t lastTime;

    void normalizeAngle() {
        if (theta > PI)  theta -= TWO_PI_F;
        if (theta < -PI) theta += TWO_PI_F;
    }
};
