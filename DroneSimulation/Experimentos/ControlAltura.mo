within DroneSimulation.Experimentos;

model ControlAltura "Experimento de regulacion de altura a 2 m sin disturbios"
  extends DroneSimulation.DroneBattery(
    x_ref = 0,
    y_ref = 0,
    z_ref = 2,
    V_wind = 0,
    V_gust = 0,
    sigma_turb = 0,
    m_payload = 0
  );
end ControlAltura;
