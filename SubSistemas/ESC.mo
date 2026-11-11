within ModeloDrone.SubSistemas;

block ESC "Equivalent speed controller: rotor speed command -> motor voltage (4 rotors)"
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  import SI = Modelica.Units.SI;

  // Motor model (used for the feed-forward term)
  parameter SI.Resistance R "Armature resistance";
  parameter Real ke(unit = "V.s/rad") "Back-EMF constant";
  parameter Real kt(unit = "N.m/A") = ke "Torque constant";
  parameter Real bm(unit = "N.m.s/rad") "Viscous friction coefficient";
  parameter Real km(unit = "N.m.s2/rad2") "Propeller torque coefficient";
  // PI speed loop
  parameter Real kp(unit = "V.s/rad") = 0.05 "Proportional gain";
  parameter Real ki(unit = "V/rad") = 0.75 "Integral gain";
  parameter Boolean steadyStateInit = true "true: integrators start in steady state";

  RealInput w_ref[4](each unit = "rad/s") "Rotor speed commands" annotation(
    Placement(transformation(extent = {{-140, -20}, {-100, 20}})));
  RealInput w_m[4](each unit = "rad/s") "Measured rotor speeds" annotation(
    Placement(transformation(origin = {70, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  RealInput V_bat(unit = "V") "Available battery voltage" annotation(
    Placement(transformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  RealOutput V[4](each unit = "V") "Voltage applied to each motor, 0 <= V <= V_bat" annotation(
    Placement(transformation(extent = {{100, -10}, {120, 10}})));

  Real e[4](each unit = "rad/s") "Speed error";
  Real x[4](each unit = "rad", each start = 0) "Integral of the speed error";
  SI.Voltage V_ff[4] "Model-based feed-forward voltage";
  SI.Voltage V_u[4] "Unsaturated voltage";

initial equation
  if steadyStateInit then
    der(x) = zeros(4);
  else
    x = zeros(4);
  end if;

equation
  for j in 1:4 loop
    e[j] = w_ref[j] - w_m[j];
    // steady state of the motor at w = w_ref: V = ke w + R (km w^2 + bm w)/kt
    V_ff[j] = ke*w_ref[j] + R/kt*(km*w_ref[j]^2 + bm*w_ref[j]);
    V_u[j] = V_ff[j] + kp*e[j] + ki*x[j];
    V[j] = min(max(V_u[j], 0), V_bat);
    // conditional integration (anti-windup)
    der(x[j]) = if (V_u[j] > V_bat and e[j] > 0) or (V_u[j] < 0 and e[j] < 0) then 0 else e[j];
  end for;

  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(textColor = {0, 0, 127}, extent = {{-60, 30}, {60, -30}}, textString = "ESC"), Text(textColor = {0, 0, 127}, extent = {{-95, 20}, {-55, -20}}, textString = "ω*"), Text(textColor = {0, 0, 127}, extent = {{55, 20}, {95, -20}}, textString = "V"), Text(textColor = {0, 0, 127}, extent = {{40, 95}, {100, 60}}, textString = "ω"), Text(textColor = {0, 0, 127}, extent = {{-40, -60}, {40, -95}}, textString = "Vbat")}),
    Documentation(info = "<html><p>Equivalent motor speed controller (ESC). It closes a PI loop on each rotor speed with a model-based feed-forward term and saturates the output to the battery voltage, 0 &le; V<sub>i</sub> &le; V<sub>bat</sub>.</p></html>"));
end ESC;
