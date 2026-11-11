# ModeloDrone

Leer TODO.md para cosas por hacer.

## Arquitectura del Sistema

```
Señal de control → Control Posición (P) → Control Velocidad (PI) → Fuerza (F=ma+mg) → invAllocMat
                                                                   ↑
Sensores ←─────────────────────────────────────────────────────────┘

invAllocMat → ESC (×4) → Motor Eléctrico (×4) → AllocMat → Fuerza/Torque → Body
                      ↑                           ↑
                      │                           │
              Batería 3S (11.1V, 2.2Ah) ←─────────┘ (corriente total)

WindDisturbance → Fuerza/Torque (perturbaciones)
```

### Cadena de control
1. **Control Posición (P)**: setpoint → aceleración deseada
2. **Control Velocidad (PI)**: aceleración deseada + velocidad medida → aceleración real
3. **ForceCompute**: F = mass × (a + g)
4. **invAllocationMatrix**: F_z, τ → velocidades de rotor comando (w_cmd)
5. **ESC**: w_cmd + V_battery → voltaje PWM (V_out)
6. **Motor Eléctrico**: V_in → velocidad real (w) + corriente (I)
7. **AllocationMatrix**: w → F_z, τ[3]
8. **ForceAndTorque → Body**: aplica fuerzas y torques al cuerpo rígido

### Sistema eléctrico
- **Batería**: 3S LiPo, V_nominal=11.1V, capacidad=2.2Ah, SoC, V_terminal
- **ESC** (×4): convierte comando de velocidad en voltaje PWM, eficiencia 0.95
- **Motor** (×4): modelo electromecánico, Ke = 11.1/910 V·s/rad, I_max=10A, tau=0.05s
- El voltaje de batería afecta directamente al empuje máximo disponible

### Parámetros físicos
| Parámetro | Valor | Unidad |
|-----------|-------|--------|
| kf | 0.61 / 910² ≈ 7.37e-7 | N·s²/rad² |
| km | kf × 0.015 ≈ 1.10e-8 | N·m·s²/rad² |
| mass | 0.2 | kg |
| Ixx, Iyy | 0.0082 | kg·m² |
| Izz | 0.0149 | kg·m² |
| l | 0.2 | m |
| w_max | 1000 | rad/s |

### Ganancias de control (sintonizadas)
| Controlador | Tipo | Ganancias |
|-------------|------|-----------|
| pos_controller3D | P | k={2,2,2}, Ti={0.5,0.5,0.5}, Td={0.05,0.05,0.05}, yMax={15,15,15} |
| vel_controller3D | PI | k={2,2,2}, Ti={0.5,0.5,0.5}, Td={0.05,0.05,0.05}, yMax={15,15,15} |
| attitudeController | Geométrico SO(3) | kR=200, kW=40 |

### Experimentos
| Experimento | Descripción | Stop time |
|-------------|-------------|-----------|
| ControlAltura | Hover estable a 2 m (línea base) | 10 s |
| ControlAlturaPerturb | Hover a 2 m con disturbios de viento | 10 s |
| ControlAlturaPayload | Hover a 2 m con payload 50 g | 10 s |
| RespuestaEscalonPosicion | Escalón a posición (2,3,2) | 15 s |
| BateriaDescarga | Descarga de batería en hover a 2 m | 300 s |
| PruebaMotores | Matriz de asignación open-loop | 5 s |
