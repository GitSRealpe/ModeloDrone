within ModeloDrone.Experimentos;

model AutonomiaHover "Q2.2: Tiempo de vuelo en hover (z=1m) hasta agotar batería"
  extends Drone(
    x_ref = 0,
    y_ref = 0,
    z_ref = 1,
    yaw_ref = 0,
    m_payload = 0,
    c_d = 0,
    F_wind = {0, 0, 0},
    t_wind = 0
  );
end AutonomiaHover;
