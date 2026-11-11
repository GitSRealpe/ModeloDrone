within DroneSimulation.Experimentos;

model PruebaMotores "Experimento de verificacion del mezclador de motores"
  extends DroneSimulation.DroneBattery(
    x_ref = 0,
    y_ref = 0,
    z_ref = 2,
    V_wind = 0,
    V_gust = 0,
    sigma_turb = 0,
    m_payload = 0
  );
end PruebaMotores;
