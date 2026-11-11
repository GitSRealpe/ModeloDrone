within DroneSimulation.Experimentos;

model ControlAlturaPerturb "Experimento de regulacion de altura a 2 m con disturbios de viento"
  extends DroneSimulation.DroneBattery(
    x_ref = 0,
    y_ref = 0,
    z_ref = 2,
    V_wind = 3,
    V_gust = 3,
    t_gust = 5,
    sigma_turb = 0.5,
    m_payload = 0
  );
end ControlAlturaPerturb;
