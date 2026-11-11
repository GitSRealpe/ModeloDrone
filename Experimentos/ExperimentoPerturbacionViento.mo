within ModeloDrone.Experimentos;

model ExperimentoPerturbacionViento "Estacionario a 2 m con perturbacion de viento"
  // Simula un viento lateral como disturbio sobre la velocidad del dron.
  // El viento se modela como un escalon en la medicion de velocidad,
  // para verificar que el controlador de velocidad rechace el disturbio.
  Drone drone annotation(
    Placement(transformation(origin = {-2, -2}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Step wind_step(
    startTime = 2.0,
    height = 1.0,
    offset = 0.0) annotation(
    Placement(transformation(origin = {-220, 40}, extent = {{-10, -10}, {10, 10}})));
equation
  // Conectar el disturbio a la velocidad medida si se expone la entrada;
  // de momento se deja como bloque fuente independiente para observar en el plot.
  // connect(wind_step.y, drone.vel_controller3D.u_m[1]);
end ExperimentoPerturbacionViento;
