within ModeloDrone.Experimentos;

model TiempoEnergiaPos "Q2.5: Tiempo y energía para alcanzar posición"
  extends Drone(
    x_ref = 2,
    y_ref = 2,
    z_ref = 2,
    yaw_ref = 0,
    m_payload = 0,
    c_d = 0,
    F_wind = {0, 0, 0},
    t_wind = 0
  );
end TiempoEnergiaPos;
