within ModeloDrone.Experimentos;

model RespuestaEscalonAltitud "Respuesta a escalon de altitud (0 m -> 2 m)"
  Drone drone annotation(
    Placement(transformation(origin = {-2, -2}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Step altitude_step(
    startTime = 1.0,
    height = 2.0,
    offset = 0.0) annotation(
    Placement(transformation(origin = {-220, 40}, extent = {{-10, -10}, {10, 10}})));
equation
  // Nota: conectar altitude_step al setpoint de posicion Z una vez expuesto.
end RespuestaEscalonAltitud;
