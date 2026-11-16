within ModeloDrone.Experimentos;

model Posicion3Ejes "Q2.4: Posición y velocidad en 3 ejes (ensayo 1 de 5)"
  extends Drone(
    x_ref = 4,
    y_ref = 4,
    z_ref = 4,
    yaw_ref = 0,
    m_payload = 0,
    c_d = 0,
    F_wind = {0, 0, 0},
    t_wind = 0
  );
end Posicion3Ejes;
