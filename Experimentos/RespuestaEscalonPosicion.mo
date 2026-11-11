within ModeloDrone.Experimentos;

model RespuestaEscalonPosicion "Respuesta a escalón en posición (2,3,2)"
  Drone drone(
    z_ref = 2,      // Altitud objetivo: 2 m
    x_ref = 2,      // Posición X objetivo: 2 m
    y_ref = 3,      // Posición Y objetivo: 3 m
    payload_mass = 0
  ) annotation(
    Placement(transformation(origin = {-2, -2}, extent = {{-10, -10}, {10, 10}})));
equation

end RespuestaEscalonPosicion;
