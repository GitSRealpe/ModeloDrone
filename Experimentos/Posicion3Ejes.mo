within ModeloDrone.Experimentos;

model Posicion3Ejes "Q2.4: desplazamiento combinado en los tres ejes (x=1, y=0.5, z=1)"
  extends Drone(x_ref = 1, y_ref = 0.5, z_ref = 1, yaw_ref = 0);
end Posicion3Ejes;
