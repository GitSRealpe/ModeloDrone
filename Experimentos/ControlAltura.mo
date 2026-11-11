within ModeloDrone.Experimentos;

model ControlAltura "Experimento de regulacion de altura a 2 m sin disturbios"
  Drone drone(
    z_ref = 2,      // Altitud objetivo: 2 m
    x_ref = 0,      // Posicion X objetivo: 0 m
    y_ref = 0       // Posicion Y objetivo: 0 m
  ) annotation(
    Placement(transformation(origin = {-2, -2}, extent = {{-10, -10}, {10, 10}})));
equation

end ControlAltura;
