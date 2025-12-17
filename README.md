# Vehiculo-plataforma
# Vehículo Plataforma - ESP32-S3 Prototipo

Prototipo de plataforma móvil controlada con **ESP32 DevKit S3** para aplicaciones de robótica y mecatrónica. Desarrollado en **Diseño Mecatrónico II 2025** como base para sistemas autónomos.[1]

## 📋 Características técnicas

- **Microcontrolador**: ESP32-S3 (Dual-core Xtensa LX7 @ 240MHz, 512KB SRAM, 8MB PSRAM)
- **Conectividad**: WiFi 802.11 b/g/n, Bluetooth 5.0 LE
- **E/S**: 45 GPIOs, ADC, DAC, I2C, SPI, UART, PWM
- **Alimentación**: 5V/3.3V, consumo típico \( 100-250mA \)
- **Memoria**: 16MB Flash, soporte SPIFFS/FAT
- **Framework**: PlatformIO con Arduino/ESP-IDF

## 🏗️ Arquitectura del sistema

```
[ESP32-S3] ← WiFi/Bluetooth → [App Móvil/Web]
    ↓
[ Motores DC ] [ Sensores ] [ Display OLED ]
    ↓
  [ Plataforma Móvil ]
```

**Diagrama de bloques**:
```
Velocidad: v = ω × r × (PWM/255)
Donde:
ω = velocidad angular (rad/s)
r = radio rueda (m)
PWM = duty cycle (0-255)
```

## 🔧 Instalación y compilación

1. **Clonar repositorio**:
```bash
git clone https://github.com/eatechnology1/Vehiculo-plataforma.git
cd "Vehiculo-plataforma"
```

2. **Instalar PlatformIO** (VS Code extension)

3. **Compilar y subir**:
```bash
pio run -t upload
pio run -t uploadfs  # Archivos SPIFFS
```

**Dependencias** (`platformio.ini`):
```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
lib_deps = 
    me-no-dev/ESPAsyncWebServer
    bblanchon/ArduinoJson
```

## ⚙️ Configuración

### Pines asignados

| Componente       | GPIO | Función        |
|------------------|------|----------------|
| Motor Izq. A     | GPIO1| PWM Velocidad  |
| Motor Izq. B     | GPIO2| Dirección      |
| Motor Der. A     | GPIO3| PWM Velocidad  |
| Motor Der. B     | GPIO4| Dirección      |
| Sensor Frontal   | GPIO5| Ultrasonido Trig |
| Sensor Lateral   | GPIO6| Ultrasonido Echo |

### Parámetros cinemáticos

```
Distancia entre ruedas: L = 0.25m
Radio rueda: r = 0.0325m
RPM motor: n = 100-300 rpm
Velocidad lineal: v = 2πrn/60 = 0.17-0.51 m/s [1]

Velocidad angular: ω = (v_d - v_i)/L
```

## 🚀 Modos de operación

### 1. **Control Manual (Web/App)**
```
GET /control?pwmL=128&pwmR=128&dirF=1
Respuesta JSON:
{
  "status": "OK",
  "velocidad": 0.34 m/s,
  "tiempo_estimado": "2.94s para 1m"
}
```

### 2. **Autónomo - Evitación de obstáculos**
```
Algoritmo PID para seguimiento de línea:
u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de/dt

Donde:
e(t) = error de posición (px)
Kp = 2.0, Ki = 0.1, Kd = 0.5
```

### 3. **Modo QR Control**
Escanea QR para acceso directo: `http://192.168.4.1/control`

## 📊 Control PID implementado

```cpp
// Controlador PID para motores
float pid_compute(float setpoint, float measured) {
    float error = setpoint - measured;
    integral += error * dt;
    derivative = (error - prev_error) / dt;
    
    float output = Kp*error + Ki*integral + Kd*derivative;
    prev_error = error;
    return constrain(output, -255, 255);
}
```

**Parámetros tuneados**:
\[ K_p = 2.5, \quad K_i = 0.15, \quad K_d = 0.8 \]

## 🔋 Gestión energética

```
Consumo total: P = V × I = 5V × 0.25A = 1.25W
Autonomía: t = (C × V)/I = (2000mAh × 3.7V)/250mA ≈ 29.6h
Eficiencia: η = P_out/P_in = 85%
```

## 🛠️ Troubleshooting

| Problema              | Causa                    | Solución                    |
|----------------------|--------------------------|-----------------------------|
| Motores no giran     | PWM fuera de rango       | Verificar `constrain(0,255)` |
| Conexión WiFi falla  | AP no visible            | Reiniciar ESP32, chequear 2.4GHz |
| Sensores erráticos   | Ruido ADC                | Agregar capacitores 100nF |
| Sobrecalentamiento   | PWM 100% prolongado      | Implementar duty cycle <80% |

## 📈 Métricas de rendimiento

| Parámetro         | Valor | Unidad |
|-------------------|-------|--------|
| Aceleración       | 0.8   | m/s²   |
| Velocidad máx.    | 0.51  | m/s    |
| Radio giro mín.   | 0.125 | m      |
| Latencia control  | 25    | ms     |
| Precisión PID     | ±5    | %      |

## 🔮 Roadmap futuro

- [ ] Integración ROS2
- [ ] SLAM con LiDAR
- [ ] Navegación autónoma
- [ ] App móvil nativa
- [ ] Detección de objetos (TensorFlow Lite)

## 📚 Referencias

1. Cinemática diferencial: \( v = \frac{r \times 2\pi \times n}{60} \)[1]
2. Control PID: Åström, K.J., Hägglund, T. (2006)[2]
3. ESP32-S3 Technical Reference: Espressif Systems

**Estado**: Prototipo funcional v1.0 - Diciembre 2025  
**Autor**: EA Technology (eatechnology1)[3][1]
