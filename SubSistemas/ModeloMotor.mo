within ModeloDrone.SubSistemas;

model ModeloMotor "Motor eléctrico: convierte voltaje en velocidad y consume corriente"
  import Modelica.Blocks.Interfaces.RealOutput;
  import Modelica.Blocks.Interfaces.RealInput;
  import SI = Modelica.Units.SI;

  parameter Real w_max(unit = "rad/s") = 1000 "Velocidad máxima del motor [rad/s]";
  parameter Real Ke = 11.1 / 910 "Constante contra-electromotriz [V*s/rad]";
  parameter Real I_max = 10.0 "Corriente máxima del motor [A]";
  parameter Real tau_motor = 0.05 "Constante de tiempo mecánica [s]";

  RealInput V_in(unit = "V") "Voltaje de entrada del ESC [V]" annotation(
    Placement(transformation(origin = {0, -76}, extent = {{-140, 55}, {-100, 95}})));

  RealOutput w(unit = "rad/s") "Velocidad real del rotor [rad/s]" annotation(
    Placement(transformation(origin = {240, -76}, extent = {{-140, 55}, {-100, 95}})));
  RealOutput I(unit = "A") "Corriente consumida por el motor [A]" annotation(
    Placement(transformation(origin = {240, -20}, extent = {{-140, 55}, {-100, 95}})));

protected
  Real w_int(start = 0) "Estado interno de velocidad";
  Real w_efectivo "Velocidad efectiva limitada por voltaje";

equation
  w_efectivo = V_in / Ke;
  der(w_int) = (w_efectivo - w_int) / tau_motor;
  w = min(max(w_int, -w_max), w_max);
  I = I_max * (abs(w) / w_max)^2;

  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {
      Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}),
      Text(textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"),
      Text(origin = {-114, 4}, textColor = {0, 0, 127}, extent = {{5, 20}, {95, -20}}, textString = "motor"),
      Text(origin = {120, -82}, textColor = {0, 0, 127}, extent = {{-95, 45}, {-5, 5}}, textString = "w")
    }),
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})));
end ModeloMotor;
