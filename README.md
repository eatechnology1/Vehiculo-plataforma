# 🚗 Plataforma Robótica ESP32-S3

**Plataforma móvil diferencial** con **odometría por encoders**, **control UDP WiFi** y **arquitectura modular**. Desarrollada para **Diseño Mecatrónico II 2025-2**.[1][2]

## 🏗️ Arquitectura del sistema

```
[WiFi UDP] → v_cmd(m/s), w_cmd(rad/s) → [Cinemática Inversa] → ωL, ωR (rad/s)
                                                            ↓
[Encoders x5] ← ωL_meas, ωR_meas ← [ISRs Cuadratura]     [Motores TB6612FNG x3]
                                                            ↓
                                                    [Odometría Euler] → x,y,θ
```

**Cinemática diferencial**:[3]
```
ωL = (v - 0.5×w×TRACK_WIDTH) / WHEEL_RADIUS = 0.51 rad/s
ωR = (v + 0.5×w×TRACK_WIDTH) / WHEEL_RADIUS = 0.77 rad/s

TRACK_WIDTH = 0.132m, WHEEL_RADIUS = 0.026m [file:48]
```

## 🖥️ Hardware

### Pines críticos[2]
| Función | GPIO | Canal PWM |
|---------|------|-----------|
| **Motor Izq** | PWM_L=9, IN1_L=11, IN2_L=10 | CH0 |
| **Motor Der** | PWM_R=12, IN1_R=14, IN2_R=13 | CH1 |
| **Plataforma** | PWM_P=18, IN1_P=8, IN2_P=3 | CH2 |
| **STBY Driver** | 46 | - |

**Encoders (PPR=4200)**:
- Izquierda: ENC_L1(4,1), ENC_L2(41,40)
- Derecha: ENC_R1(2,42), ENC_R2(39,38)
- Plataforma: ENC_P(37,36)

**PWM**: 20kHz, 8-bit (0-255)[4]

## ⚙️ Software

**Frecuencia control**: 100Hz (\( \Delta t = 10ms \))[1]

### Módulos principales
| Módulo | Función | Fichero |
|--------|---------|---------|
| **MotorManager** | TB6612FNG x3 (u∈[-1,1]) | motor_driver.h [4] |
| **EncoderManager** | 5 encoders cuadratura | encoder.h [5] |
| **Kinematics** | Odometría Euler | kinematics.h [3] |
| **PID** | Control velocidad (anti-windup) | pid.h [6] |
| **WifiComm** | UDP "v:0.3,w:-0.5" | wifi_comm.h [7] |

### Protocolo UDP[7][1]
```
Formato: "v:0.30,w:-0.50"
Puerto: 8888
Timeout: 5s → v_cmd=w_cmd=0
Límites: V_MAX=0.6m/s, W_MAX=2.0rad/s
```

## 🔧 Compilación e instalación

```bash
# Requisitos: PlatformIO + ESP32-S3-DevKitC-1
pio lib install "ESP32 Arduino Core"
pio run --target upload --environment esp32-s3-devkitc-1
```

**WiFi**:[2]
```
SSID: "Calderon Ramon"
Pass: "Lina.Alex.321"
IP: Ver Serial Monitor (115200 baud)
```

## 📊 Rendimiento

| Parámetro | Valor | Unidad |
|-----------|-------|--------|
| **Frecuencia control** | 100 | Hz |
| **Vel. máx. lineal** | 0.6 | m/s |
| **Vel. máx. angular** | 2.0 | rad/s |
| **Resolución encoder** | 4200 | PPR |
| **Odometría** | Euler 1er orden | - |

**Debug Serial** (cada 500ms):
```
[MEAS] ωL=0.512 ωR=0.774 | uL=0.65 uR=0.98
[ODO] x=0.234 y=0.156 θ=1.23 | v=0.32 w=-0.45
```

## 🧮 Algoritmo principal[1]

```cpp
// Bucle 100Hz
encoders.update();
kine.update(ωL_meas, ωR_meas);

// Cinemática inversa
ωL_ref = (v_cmd - 0.5*w_cmd*TRACK_WIDTH)/WHEEL_RADIUS;
ωR_ref = (v_cmd + 0.5*w_cmd*TRACK_WIDTH)/WHEEL_RADIUS;

// Normalización [-1,1]
uL = constrain(ωL_ref/(V_MAX/WHEEL_RADIUS), -1, 1);
motors.setLeft(uL);
```

## 🚀 Uso

1. **Conectar ESP32-S3** → USB
2. **Compilar/Upload** → PlatformIO
3. **Monitor Serial** → IP asignada
4. **Enviar UDP** desde PC/móvil:
```bash
echo "v:0.3,w:0.0" | nc -u IP_ESP32 8888
```

## 🔮 Próximos pasos

- [ ] **Control PID ruedas** (usar pid.h)
- [ ] **Fusión IMU** (odometría robusta)
- [ ] **App Android/iOS**
- [ ] **SLAM básico**
- [ ] **Web Dashboard**

## 📚 Referencias

1. Cinemática diferencial: \( v = 0.5(v_R + v_L), w = \frac{v_R - v_L}{L} \)[3]
2. Control PID: Anti-windup clamping[6]
3. TB6612FNG: PWM 20kHz, 8-bit[4]

**Autor**: Alexander Calderon - EA Technology (eatechnology1)  
**v1.0**: Diciembre 2025[8][2][1]
