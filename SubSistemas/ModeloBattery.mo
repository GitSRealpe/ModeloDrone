within ModeloDrone.SubSistemas;

model ModeloBattery "Batería LiPo 3S con estado de carga y voltaje terminal"
  import Modelica.Blocks.Interfaces.RealOutput;
  import Modelica.Blocks.Interfaces.RealInput;
  import SI = Modelica.Units.SI;

  parameter SI.Voltage V_nominal = 11.1 "Voltaje nominal de la batería 3S [V]";
  parameter SI.Charge capacity = 2.2 "Capacidad de la batería [Ah]";
  parameter Real V_full = 12.6 "Voltaje de carga completa [V]";
  parameter Real V_empty = 9.0 "Voltaje de descarga profunda [V]";

  RealInput I_total(unit = "A") "Corriente total consumida por los motores [A]" annotation(
    Placement(transformation(origin = {0, 0}, extent = {{-140, 55}, {-100, 95}})));

  RealOutput V_terminal(unit = "V") "Voltaje terminal de la batería [V]" annotation(
    Placement(transformation(origin = {240, -20}, extent = {{-100, 10}, {-80, 30}})));
  RealOutput SoC(unit = "1") "Estado de carga [0..1]" annotation(
    Placement(transformation(origin = {240, 60}, extent = {{-100, 10}, {-80, 30}})));

protected
  Real SoC_int(start = 1.0) "Estado de carga integrado";

equation
  der(SoC_int) = -I_total / (capacity * 3600.0);
  SoC = min(1.0, max(0.0, SoC_int));
  V_terminal = V_nominal * SoC;

  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {
      Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}),
      Text(textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"),
      Text(origin = {-90, 4}, textColor = {0, 0, 127}, extent = {{5, 20}, {95, -20}}, textString = "batería 3S")
    }),
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})));
end ModeloBattery;
