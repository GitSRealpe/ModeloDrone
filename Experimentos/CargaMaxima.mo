within ModeloDrone.Experimentos;

model CargaMaxima "Q2.1: Carga máxima que puede levantar el dron"
  extends Drone(
    x_ref = 10,
    y_ref = 10,
    z_ref = 10,
    yaw_ref = 0,
    m_payload = 0.09,
    c_d = 0,
    F_wind = {0, 0, 0},
    t_wind = 0
  );
end CargaMaxima;
