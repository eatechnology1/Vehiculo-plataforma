# ROBOT MÓVIL ESP32-S3 - DOCUMENTACIÓN COMPLETA

## Tabla de Contenidos
1. [Arquitectura General del Sistema](#arquitectura-general)
2. [Componentes Hardware](#componentes-hardware)
3. [Arquitectura Software](#arquitectura-software)
4. [Flujo de Datos](#flujo-de-datos)
5. [Configuración del Sistema](#configuración)
6. [Protocolo de Comunicación](#protocolo-comunicación)
7. [Seguridad y Robustez](#seguridad)
8. [Calibración y Optimización](#calibración)
9. [Troubleshooting](#troubleshooting)

---

## 1. Arquitectura General del Sistema

### Descripción General
Este proyecto implementa un robot móvil diferencial controlado por microcontrolador ESP32-S3. El sistema está diseñado para ser controlado remotamente vía WiFi mediante comandos UDP, integrando sensores de posición (encoders) para odometría y control de motores para navegación.

### Características Principales
- **Control Diferencial**: Dos orugas motrices independientes
- **Control Remoto**: Comunicación WiFi con protocolo UDP
- **Odometría Integrada**: Cálculo de posición y orientación en tiempo real
- **Frecuencia de Control**: 100 Hz para respuesta rápida
- **Seguridad**: Timeout automático y paradas de emergencia

### Aplicaciones
- Plataforma de investigación en robótica móvil
- Desarrollo de algoritmos de control
- Prototipo para vehículos autónomos
- Sistema educativo para mecatrónica

---

## 2. Componentes Hardware 

### 2.1 Microcontrolador ESP32-S3
- **Modelo**: ESP32-S3-WROOM-1
- **Función**: Unidad de control principal
- **Características**:
  - Procesador dual-core de 32 bits
  - Conectividad WiFi integrada
  - 34 GPIO programables
  - Múltiples interfaces de comunicación (I2C, SPI, UART)
  - ADC de 12 bits para lecturas analógicas

### 2.2 Drivers de Motor TB6612FNG
- **Función**: Control bidireccional de motores DC
- **Configuración**:
  - **Motor Izquierdo**: PWM_L, IN1_L, IN2_L (Pines 9, 11, 10)
  - **Motor Derecho**: PWM_R, IN1_R, IN2_R (Pines 12, 14, 13)
  - **Motor Plataforma**: PWM_P, IN1_P, IN2_P (Pines 18, 8, 3)
  - **Standby**: Pin 46 (control de habilitación)

### 2.3 Sistema de Encoders
- **Tipo**: Encoders incrementales en cuadratura
- **Resolución**: 4200 pulsos por revolución (PPR)
- **Configuración**:
  - **Oruga Izquierda**: 
    - Encoder L1: Pines 4 (A), 1 (B)
    - Encoder L2: Pines 41 (A), 40 (B)
  - **Oruga Derecha**:
    - Encoder R1: Pines 2 (A), 42 (B)
    - Encoder R2: Pines 39 (A), 38 (B)
  - **Plataforma**: Pines 37 (A), 36 (B)

### 2.4 Parámetros Mecánicos
- **Ancho de Vía (TRACK_WIDTH)**: 0.132 m
- **Radio de Rueda (WHEEL_RADIUS)**: 0.026 m
- **Velocidad Lineal Máxima (V_MAX)**: 0.6 m/s
- **Velocidad Angular Máxima (W_MAX)**: 2.0 rad/s

---

## 3. Arquitectura Software 

### 3.1 Estructura de Módulos

#### main.cpp
- **Propósito**: Programa principal y orquestación del sistema
- **Responsabilidades**:
  - Bucle de control principal a 100 Hz
  - Integración de todos los subsistemas
  - Gestión de timing y sincronización
  - Logging y depuración

#### config.h
- **Propósito**: Configuración centralizada de hardware y parámetros
- **Contenido**:
  - Definiciones de pines GPIO
  - Constantes de control
  - Parámetros de red WiFi
  - Límites de seguridad

#### encoder.h
- **Propósito**: Gestión de sensores de posición
- **Clases**:
  - `Encoder`: Control individual de encoder
  - `EncoderManager`: Coordinación de múltiples encoders
- **Funcionalidades**:
  - Lectura de encoders en cuadratura
  - Cálculo de velocidades angulares
  - Compensación de ruido

#### motor_driver.h
- **Propósito**: Control de actuadores
- **Clases**:
  - `Motor`: Control individual de motor DC
  - `MotorManager`: Coordinación de múltiples motores
- **Funcionalidades**:
  - Control PWM bidireccional
  - Normalización de comandos
  - Paradas de seguridad

#### kinematics.h
- **Propósito**: Estimación de estado del robot
- **Clase**: `Kinematics`
- **Funcionalidades**:
  - Integración de odometría
  - Cálculo de velocidades del chasis
  - Transformación de coordenadas

#### pid.h
- **Propósito**: Control proporcional-integral-derivativo
- **Clase**: `PID`
- **Características**:
  - Derivada sobre la medición
  - Anti-windup por clamping
  - Saturación de salida

#### wifi_comm.h
- **Propósito**: Comunicación inalámbrica
- **Clase**: `WifiComm`
- **Funcionalidades**:
  - Conexión WiFi
  - Recepción de comandos UDP
  - Detección de timeout

### 3.2 Principios de Diseño

#### Modularidad
- Cada módulo tiene responsabilidades claramente definidas
- Interfaces bien establecidas entre componentes
- Facilita mantenimiento y extensión

#### Tiempo Real
- Bucle de control determinista a 100 Hz
- Priorización de tareas críticas
- Manejo seguro de interrupciones

#### Robustez
- Detección de fallos en comunicación
- Paradas automáticas por timeout
- Validación de datos de entrada

---

## 4. Flujo de Datos 

### 4.1 Secuencia de Operaciones

```
[Inicio del Sistema]
    ↓
[Inicialización WiFi]
    ↓
[Configuración Hardware]
    ↓
[BUCLE PRINCIPAL (100 Hz)]
    ↓
[Recepción de Comandos UDP]
    ↓
[Verificación de Seguridad]
    ↓
[Lectura de Encoders]
    ↓
[Actualización de Odometría]
    ↓
[Cálculo de Cinemática Inversa]
    ↓
[Control de Motores]
    ↓
[Logging y Monitoreo]
    ↓
[Retorno al Inicio del Bucle]
```

### 4.2 Flujo de Datos Detallado

#### Entrada de Comandos
1. **Aplicación Remota** → Paquete UDP
2. **ESP32** → `WifiComm.receive()`
3. **Parser** → Extracción de v, w
4. **Validación** → Límites de seguridad
5. **Almacenamiento** → `v_cmd`, `w_cmd`

#### Procesamiento de Sensores
1. **Encoders** → Pulsos de cuadratura
2. **ISR** → Actualización de contadores
3. **Cálculo** → Velocidades angulares
4. **Filtrado** → Reducción de ruido
5. **Salida** → `omegaL_meas`, `omegaR_meas`

#### Control de Movimiento
1. **Odometría** → Integración de posición
2. **Cinemática** → Velocidades de ruedas
3. **Normalización** → Comandos PWM
4. **Drivers** → Activación de motores
5. **Retroalimentación** → Medición de encoders

### 4.3 Estados del Sistema

#### Estado Inicial
- Todos los parámetros en cero
- Motores desactivados
- WiFi desconectado
- Encoders reiniciados

#### Estado Operativo
- WiFi conectado
- Recepción de comandos activa
- Bucle de control funcionando
- Logging en curso

#### Estado de Emergencia
- Timeout de comandos detectado
- Motores inmediatamente detenidos
- Sistema en modo seguro
- Requiere intervención manual

---

## 5. Configuración del Sistema 

### 5.1 Parámetros de Red WiFi

```cpp
#define WIFI_SSID     "Tu_Red_WiFi"
#define WIFI_PASSWORD "Tu_Password"
#define UDP_PORT      8888
```

**Configuración**:
- SSID: Nombre de la red WiFi
- Password: Contraseña de acceso
- Puerto UDP: Puerto para comandos (por defecto 8888)

### 5.2 Parámetros de Control

```cpp
#define V_MAX     0.6f    // [m/s] Velocidad lineal máxima
#define W_MAX     2.0f    // [rad/s] Velocidad angular máxima
#define CMD_TIMEOUT_MS  5000  // [ms] Timeout de comandos
```

### 5.3 Parámetros de Hardware

```cpp
#define TRACK_WIDTH   0.132f   // [m] Distancia entre ruedas
#define WHEEL_RADIUS  0.026f   // [m] Radio de ruedas
#define PPR           4200     // Pulsos por revolución
```

### 5.4 Configuración PWM

```cpp
#define PWM_FREQ  20000    // [Hz] Frecuencia PWM
#define PWM_RES   8        // bits de resolución
#define PWM_MAX   255      // Valor máximo de duty cycle
```

---

## 6. Protocolo de Comunicación 

### 6.1 Formato de Mensaje

**Formato**: `"v:<velocidad_lineal>,w:<velocidad_angular>"`

**Ejemplos**:
- `"v:0.25,w:0.50"` - Avanzar 0.25 m/s y girar 0.50 rad/s
- `"v:-0.30,w:1.0"` - Retroceder 0.30 m/s y girar 1.0 rad/s
- `"v:0.0,w:0.0"` - Detener movimiento

### 6.2 Unidades y Rangos

- **Velocidad Lineal (v)**: metros por segundo
  - Rango: [-V_MAX, V_MAX] = [-0.6, 0.6] m/s
- **Velocidad Angular (w)**: radianes por segundo
  - Rango: [-W_MAX, W_MAX] = [-2.0, 2.0] rad/s

### 6.3 Secuencia de Comunicación

1. **Conexión WiFi**: ESP32 se conecta a la red
2. **Escucha UDP**: Abre puerto para recibir comandos
3. **Recepción**: Lee paquetes UDP entrantes
4. **Parseo**: Extrae valores v, w del mensaje
5. **Validación**: Verifica límites de seguridad
6. **Aplicación**: Actualiza consignas de control

### 6.4 Ejemplo de Cliente Python

```python
import socket
import struct

# Configuración
ESP32_IP = "192.168.1.100"  # IP del ESP32
UDP_PORT = 8888

# Crear socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Enviar comando
v = 0.3  # 0.3 m/s hacia adelante
w = 0.5  # 0.5 rad/s hacia la izquierda
message = f"v:{v},w:{w}"
sock.sendto(message.encode(), (ESP32_IP, UDP_PORT))
```

---

## 7. Seguridad y Robustez 

### 7.1 Mecanismos de Seguridad

#### Timeout de Comandos
- **Función**: Detecta pérdida de comunicación
- **Comportamiento**: Para el robot si no hay comandos por >5 segundos
- **Implementación**: `millis() - lastCmdTime > CMD_TIMEOUT_MS`

#### Límites de Velocidad
- **Función**: Previene movimientos peligrosos
- **Límites**: Velocidades máxima y mínima configurables
- **Implementación**: `constrain(v_cmd, -V_MAX, V_MAX)`

#### Pin Standby
- **Función**: Habilitación/deshabilitación rápida de motores
- **Control**: Pin STBY (46) en config.h
- **Comportamiento**: LOW = motores deshabilitados, HIGH = habilitados

### 7.2 Validación de Datos

#### Verificación de Formato
- Parseo de comandos UDP con `sscanf`
- Validación de rangos numéricos
- Manejo de mensajes malformados

#### Robustez de Hardware
- Configuración INPUT_PULLUP en encoders
- Protección contra sobrecorriente en drivers
- Monitoreo de temperatura del ESP32

### 7.3 Recuperación de Errores

#### Reinicio del Sistema
- Watchdog software implícito en timeout
- Reinicio de conexiones WiFi
- Recuperación automática de encoders

#### Logging de Eventos
- Registro de comandos recibidos
- Monitoreo de estado del sistema
- Detección de anomalías

---

## 8. Calibración y Optimización 

### 8.1 Calibración de Parámetros

#### Medición de TRACK_WIDTH
1. Marcar posición inicial del robot
2. Ejecutar rotación de 360° exacta
3. Medir desplazamiento real vs. calculado
4. Ajustar TRACK_WIDTH hasta obtener rotación precisa

#### Medición de WHEEL_RADIUS
1. Ejecutar recorrido recto conocido (ej. 1 metro)
2. Comparar distancia real vs. calculada por odometría
3. Ajustar WHEEL_RADIUS para minimizar error

#### Calibración de Encoders (PPR)
1. Rotar rueda exactamente una revolución
2. Contar pulsos reales del encoder
3. Comparar con valor PPR configurado
4. Ajustar hasta obtener correspondencia exacta

### 8.2 Optimización del Control

#### Tuning de PID (Futuro)
```cpp
// Ganancias recomendadas para inicio
PID pidLeft(0.5f, 0.1f, 0.05f);   // Motor izquierdo
PID pidRight(0.5f, 0.1f, 0.05f);  // Motor derecho
```

#### Filtrado de Señales
- **Filtro Promedio Móvil**: Para reducir ruido en encoders
- **Filtro de Kalman**: Para estimación más robusta de velocidad
- **Butterworth**: Para suavizar señales de control

### 8.3 Análisis de Rendimiento

#### Métricas de Calidad
- **Precisión de Odometría**: Error de posición por distancia recorrida
- **Respuesta del Control**: Tiempo de estabilización a nuevas consignas
- **Estabilidad**: Oscilaciones y overshoot en seguimiento

#### Herramientas de Medición
- Sistema de captura de movimiento externo
- GPS diferencial para pruebas al aire libre
- Logger de datos para análisis post-proceso

---

## 9. Troubleshooting

### 9.1 Problemas de Conexión WiFi

#### Síntoma: ESP32 no se conecta
**Posibles Causas**:
- SSID o contraseña incorrectos
- Red WiFi fuera de alcance
- Interferencia electromagnética

**Soluciones**:
1. Verificar credenciales en `config.h`
2. Comprobar intensidad de señal
3. Reiniciar router WiFi
4. Verificar configuración de seguridad de red

#### Síntoma: Conexión inestable
**Posibles Causas**:
- Interferencia de otros dispositivos
- Canal WiFi congestionado
- Alimentación insuficiente

**Soluciones**:
1. Cambiar canal WiFi en router
2. Usar alimentación externa estable
3. Verificar conexiones de antena

### 9.2 Problemas de Control

#### Síntoma: Robot no responde a comandos
**Posibles Causas**:
- Pin STBY en estado LOW
- Drivers de motor desconectados
- Timeout de seguridad activado

**Soluciones**:
1. Verificar pin STBY (46) en HIGH
2. Comprobar conexiones de motores
3. Enviar comando reciente para resetear timeout

#### Síntoma: Movimiento errático
**Posibles Causas**:
- Encoders mal calibrados
- Interferencia en líneas de encoders
- Valores PPR incorrectos

**Soluciones**:
1. Verificar conexiones de encoders
2. Recalibrar valor PPR
3. Verificar alimentación de encoders
4. Revisar interferencias electromagnéticas

### 9.3 Problemas de Odometría

#### Síntoma: Deriva en posición
**Posibles Causas**:
- Parámetros TRACK_WIDTH o WHEEL_RADIUS incorrectos
- Deslizamiento de ruedas
- Error en integración numérica

**Soluciones**:
1. Recalibrar parámetros geométricos
2. Verificar fricción de ruedas
3. Reducir frecuencia de control
4. Implementar fusión con sensores adicionales

#### Síntoma: Rotación imprecisa
**Posibles Causas**:
- Diferencias entre motores izquierdo y derecho
- Desbalance en peso del robot
- Error en TRACK_WIDTH

**Soluciones**:
1. Balancear peso del robot
2. Verificar igual potencia en ambos motores
3. Recalibrar TRACK_WIDTH con prueba de rotación

### 9.4 Herramientas de Diagnóstico

#### Serial Monitor
- Monitoreo en tiempo real de variables
- Registro de comandos recibidos
- Estado de conexiones y sensores

#### Comandos de Prueba
```cpp
// Comandos de prueba para enviar vía UDP:
// "v:0.3,w:0.0" - Avance recto
// "v:0.0,w:1.57" - Rotación 90°
// "v:0.0,w:0.0" - Parada
```

#### LEDs de Diagnóstico
- LED WiFi: Estado de conexión
- LED de actividad: Recepción de comandos
- LED de error: Fallos del sistema

---

## Conclusión

Este sistema de robot móvil ESP32-S3 proporciona una plataforma robusta y extensible para investigación y desarrollo en robótica móvil. La arquitectura modular, los mecanismos de seguridad integrados y la documentación completa facilitan tanto el uso inmediato como futuras mejoras y extensiones.

Para soporte técnico adicional o consultas sobre implementación, consulte los archivos fuente individuales que contienen documentación técnica detallada de cada componente.

---

**Fecha de Documentación**: Diciembre 2025  
**Versión del Sistema**: 1.0  
**Autor**: Alexander Calderon - EA Technology (eatechnology1)  
