within ModeloDrone.Experimentos;

model ControlAltura "Q2.4: regulación de altura a z = 2 m sin disturbios"
  extends Drone(x_ref = 0, y_ref = 0, z_ref = 2, yaw_ref = 0);
end ControlAltura;
