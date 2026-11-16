within ModeloDrone.SubSistemas;

model Bateria "Battery as an energy reservoir with constant voltage until empty"
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  import SI = Modelica.Units.SI;

  parameter SI.Voltage V_nom = 7.4 "Nominal voltage";
  parameter Real C_Ah(unit = "A.h") = 0.5 "Nominal capacity";
  parameter SI.Energy E_0 = V_nom*C_Ah*3600 "Initial (nominal) stored energy";

  RealInput V[4](each unit = "V") "Motor voltages" annotation(
    Placement(transformation(origin = {-50, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  RealInput i[4](each unit = "A") "Motor currents" annotation(
    Placement(transformation(origin = {50, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  RealOutput V_bat(unit = "V") "Available voltage" annotation(
    Placement(transformation(extent = {{-100, -10}, {-120, 10}})));
  RealOutput SOC(unit = "1") "State of charge E/E_0" annotation(
    Placement(transformation(extent = {{100, -10}, {120, 10}})));

  SI.Energy E(start = E_0, fixed = true) "Stored energy";
  SI.Power P "Electrical power drawn by the motors";
  Boolean empty = E <= 0 "Battery depleted -> flight ends";

equation
  P = sum(V[j]*i[j] for j in 1:4);
  der(E) = -P;
  V_bat = if empty then 0 else V_nom;
  SOC = E/E_0;

  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-150, -110}, {150, -150}}, textString = "%name"), Rectangle(lineColor = {0, 0, 127}, fillColor = {170, 213, 255}, fillPattern = FillPattern.Solid, extent = {{-60, 30}, {50, -30}}), Rectangle(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{50, 12}, {62, -12}}), Text(textColor = {0, 0, 127}, extent = {{-75, 95}, {-25, 60}}, textString = "V"), Text(textColor = {0, 0, 127}, extent = {{25, 95}, {75, 60}}, textString = "i"), Text(textColor = {0, 0, 127}, extent = {{-95, -40}, {-30, -80}}, textString = "Vbat"), Text(textColor = {0, 0, 127}, extent = {{30, -40}, {95, -80}}, textString = "SOC")}),
    Documentation(info = "<html><p>Battery as an energy reservoir (informe_1, eq. 9):</p>
<p>P = &Sigma; V<sub>i</sub> i<sub>i</sub>, &nbsp; dE/dt = -P.</p><p>The voltage is held at V<sub>nom</sub> until E reaches zero. At that point it drops to 0 and the flight ends.</p></html>"));
end Bateria;
