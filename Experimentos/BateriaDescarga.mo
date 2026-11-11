within ModeloDrone.Experimentos;

model BateriaDescarga "Experimento de descarga de batería en hover a 2 m"
  Drone drone(
    z_ref = 2,      // Altitud objetivo: 2 m
    x_ref = 0,
    y_ref = 0,
    payload_mass = 0
  ) annotation(
    Placement(transformation(origin = {-2, -2}, extent = {{-10, -10}, {10, 10}})));
equation

end BateriaDescarga;
