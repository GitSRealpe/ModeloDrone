within ModeloDrone.SubSistemas;

block WindDisturbance "Wind disturbance generator for force/torque perturbation"
  import Modelica.Blocks.Interfaces.RealOutput;
  import Modelica.Blocks.Interfaces.RealInput;
  import SI = Modelica.Units.SI;

  parameter Real fx_max = 0.5 "Max disturbance force x [N]";
  parameter Real fy_max = 0.5 "Max disturbance force y [N]";
  parameter Real fz_max = 0.3 "Max disturbance force z [N]";
  parameter Real tx_max = 0.02 "Max disturbance torque x [N.m]";
  parameter Real ty_max = 0.02 "Max disturbance torque y [N.m]";
  parameter Real tz_max = 0.01 "Max disturbance torque z [N.m]";
  parameter Real f = 0.2 "Disturbance frequency [Hz]";

  RealOutput fx(unit = "N") "Disturbance force x" annotation(
    Placement(transformation(origin = {240, -100}, extent = {{-100, 10}, {-80, 30}})));
  RealOutput fy(unit = "N") "Disturbance force y" annotation(
    Placement(transformation(origin = {240, -60}, extent = {{-100, 10}, {-80, 30}})));
  RealOutput fz(unit = "N") "Disturbance force z" annotation(
    Placement(transformation(origin = {240, -20}, extent = {{-100, 10}, {-80, 30}})));
  RealOutput tx(unit = "N.m") "Disturbance torque x" annotation(
    Placement(transformation(origin = {240, 20}, extent = {{-100, 10}, {-80, 30}})));
  RealOutput ty(unit = "N.m") "Disturbance torque y" annotation(
    Placement(transformation(origin = {240, 60}, extent = {{-100, 10}, {-80, 30}})));
  RealOutput tz(unit = "N.m") "Disturbance torque z" annotation(
    Placement(transformation(origin = {240, 100}, extent = {{-100, 10}, {-80, 30}})));

equation
  fx = fx_max * sin(2 * Modelica.Constants.pi * f * time);
  fy = fy_max * cos(2 * Modelica.Constants.pi * f * time);
  fz = fz_max * sin(4 * Modelica.Constants.pi * f * time);
  tx = tx_max * sin(2 * Modelica.Constants.pi * f * time);
  ty = ty_max * cos(2 * Modelica.Constants.pi * f * time);
  tz = tz_max * sin(6 * Modelica.Constants.pi * f * time);

  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {
      Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}),
      Text(textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"),
      Text(origin = {-90, 4}, textColor = {0, 0, 127}, extent = {{5, 20}, {95, -20}}, textString = "wind")
    }),
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})));
end WindDisturbance;
