within DroneSimulation.Experimentos;

model RespuestaEscalonPosicion "Experimento de respuesta a escalon en posicion (2,3,2)"
  extends DroneSimulation.DroneBattery(
    x_ref = 2,
    y_ref = 3,
    z_ref = 2,
    V_wind = 0,
    V_gust = 0,
    sigma_turb = 0,
    m_payload = 0
  );
end RespuestaEscalonPosicion;
