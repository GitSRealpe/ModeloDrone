within ModeloDrone.Experimentos;

model AutonomiaVsCarga "Q2.3: Autonomía vs carga (mismo régimen z=1m)"
  extends Drone(
    x_ref = 0,
    y_ref = 0,
    z_ref = 1,
    yaw_ref = 0,
    m_payload = 0.09,
    c_d = 0,
    F_wind = {0, 0, 0},
    t_wind = 0
  );
end AutonomiaVsCarga;
