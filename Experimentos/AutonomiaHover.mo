within ModeloDrone.Experimentos;

model AutonomiaHover "Q2.2: tiempo de vuelo en hover hasta agotar la batería (z = 1 m)"
  extends Drone(x_ref = 0, y_ref = 0, z_ref = 1, yaw_ref = 0, m_payload = 0);
end AutonomiaHover;
