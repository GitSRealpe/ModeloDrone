within ModeloDrone.Experimentos;

model ControlAlturaPayload "Experimento de regulacion de altura a 2 m con payload adicional"
  Drone drone(
    z_ref = 2,      // Altitud objetivo: 2 m
    x_ref = 0,
    y_ref = 0,
    payload_mass = 0.05  // 50 g de payload adicional (25% del peso base)
  ) annotation(
    Placement(transformation(origin = {-2, -2}, extent = {{-10, -10}, {10, 10}})));
equation

end ControlAlturaPayload;
