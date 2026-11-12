within ModeloDrone.Experimentos;

model AutonomiaVsCarga "Q2.3: autonomía vs carga transportada (m_payload = 0.02 kg, z = 1 m)"
  extends Drone(x_ref = 0, y_ref = 0, z_ref = 1, yaw_ref = 0, m_payload = 0.02);
end AutonomiaVsCarga;
