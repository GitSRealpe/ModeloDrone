within ModeloDrone.Experimentos;

model CargaMaxima "Q2.1: carga máxima que puede levantar el dron (m_payload = 0.02 kg)"
  extends Drone(x_ref = 0, y_ref = 0, z_ref = 1, yaw_ref = 0, m_payload = 0.02);
end CargaMaxima;
