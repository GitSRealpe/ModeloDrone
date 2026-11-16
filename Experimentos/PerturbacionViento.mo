within ModeloDrone.Experimentos;

model PerturbacionViento "Q2.6: Perturbaciones en régimen permanente (escalón ±0.1N)"
  extends Drone(
    x_ref = 1,
    y_ref = 1,
    z_ref = 1,
    yaw_ref = 0,
    m_payload = 0,
    c_d = 0.1,
    F_wind = {0.0, 0.1, 0},
    t_wind = 5
  );
end PerturbacionViento;
