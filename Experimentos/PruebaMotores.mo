within ModeloDrone.Experimentos;

model PruebaMotores "Experimento open-loop de matriz de asignacion y su inversa"
  import SI = Modelica.Units.SI;
  parameter SI.Length l = 0.2;
  parameter Real kf = 0.61 / 910^2;
  parameter Real km = kf * 0.015;
  parameter Real w_max = 1000.0;

  SubSistemas.invAllocationMatrix invMat(l = l, kf = kf, km = km, w_max = w_max);
  SubSistemas.AllocationMatrix mat(l = l, kf = kf, km = km);
  Modelica.Blocks.Sources.Constant Fz_ref(k = 1.962);
  Modelica.Blocks.Sources.Constant tau_x_ref(k = 0);
  Modelica.Blocks.Sources.Constant tau_y_ref(k = 0);
  Modelica.Blocks.Sources.Constant tau_z_ref(k = 0);

equation
  connect(Fz_ref.y, invMat.F_z);
  connect(tau_x_ref.y, invMat.tau_x);
  connect(tau_y_ref.y, invMat.tau_y);
  connect(tau_z_ref.y, invMat.tau_z);
  connect(invMat.w, mat.w);
end PruebaMotores;
