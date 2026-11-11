# TODO
Cosas por hacer.

- [x] Poner valores correctos de masa e inercia en el bloque *body* en el modelo **Drone**
- [x] integrar modelo dinamicas de los motores (ModeloMotor con V_in → w)
- [x] integrar modelo electrico
  - [x] motores (ModeloMotor con Ke, I_max, tau_motor)
  - [x] bateria (ModeloBattery 3S LiPo 11.1V, 2.2Ah, SoC, V_terminal)
  - [x] ESC (ModeloESC simplificado, duty_cycle → V_out)
- [x] Experimento de respuesta a escalón (posición 2,3,2)
- [x] Experimentos con disturbios de viento y payload
- [ ] Corregir conexión de batería (completado: batería integrada en cadena de energía)
- [ ] Documentación de ganancias PID sintonizadas
- [ ] Comparación antes/despues de disturbios con métricas
