within ModeloDrone.Experimentos;

model TiempoEnergiaPos "Q2.5: tiempo y energía para alcanzar una posición (x=2, y=0, z=1.5)"
  extends Drone(x_ref = 2, y_ref = 0, z_ref = 1.5, yaw_ref = 0);
end TiempoEnergiaPos;
