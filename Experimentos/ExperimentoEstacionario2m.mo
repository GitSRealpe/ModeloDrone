within ModeloDrone.Experimentos;

model ExperimentoEstacionario2m "Estacionario (hover) a 2 m de altitud"
  // Instancia del dron con los coeficientes corregidos:
  //   kf = 0.61 / 910^2  = 7.37e-7 N*s^2/rad^2
  //   km = kf * 0.015    = 1.10e-8 N*m*s^2/rad^2
  //
  // NOTA: para apuntar a 2 m, editar Drone.mo y fijar testz.k = 2
  // (el valor por defecto es testz.k = 1 -> 1 m).
  Drone drone annotation(
    Placement(transformation(origin = {-2, -2}, extent = {{-10, -10}, {10, 10}})));
equation
end ExperimentoEstacionario2m;
