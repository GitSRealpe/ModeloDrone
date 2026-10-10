within ModeloDrone;

model Drone

  import SI = Modelica.Units.SI;
  parameter SI.Length l = 0.2 "Propeller distance to CoG";
  parameter Real kf = 1e-5 "Thrust coefficient";
  parameter Real km = 1e-7 "Torque coefficient";
  parameter Real w_max(unit = "rad/s") = 1000 "Max rotor speed";

  SubSistemas.AllocationMatrix allocMat(l=l, kf=kf, km=km) annotation(
    Placement(transformation(origin = {38, 0}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.invAllocationMatrix invAllocMat(l=l, kf=kf, km=km, w_max=w_max) annotation(
    Placement(transformation(origin = {-12, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const(k = -1)  annotation(
    Placement(transformation(origin = {-84, 60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const1(k = 0) annotation(
    Placement(transformation(origin = {-84, 26}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const11(k = 0) annotation(
    Placement(transformation(origin = {-84, -14}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const12(k = 0) annotation(
    Placement(transformation(origin = {-84, -52}, extent = {{-10, -10}, {10, 10}})));
equation

  annotation(
    uses(Modelica(version = "4.1.0")),
  Icon(graphics = {Bitmap(extent = {{-140, -140}, {140, 140}}, fileName = "modelica://ModeloDrone/media/drone_white.png")}));
end Drone;