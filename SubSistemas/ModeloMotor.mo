within ModeloDrone.SubSistemas;

model ModeloMotor "Equivalent DC motor + propeller load (one rotor)"
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  import SI = Modelica.Units.SI;

  parameter SI.Resistance R "Armature resistance";
  parameter SI.Inductance L "Armature inductance";
  parameter Real ke(unit = "V.s/rad") "Back-EMF constant";
  parameter Real kt(unit = "N.m/A") = ke "Torque constant (= ke in SI units)";
  parameter SI.Inertia Jm "Rotor inertia (motor + propeller)";
  parameter Real bm(unit = "N.m.s/rad") "Viscous friction coefficient";
  parameter Real km(unit = "N.m.s2/rad2") "Propeller torque coefficient (tau_p = km*w^2)";
  parameter Boolean steadyStateInit = true "true: start in electrical/mechanical steady state";
  parameter SI.AngularVelocity w_start = 0 "Initial speed (start guess if steadyStateInit = true)";

  RealInput V(unit = "V") "Applied (equivalent) voltage" annotation(
    Placement(transformation(extent = {{-140, -20}, {-100, 20}})));
  RealOutput w(unit = "rad/s", start = w_start) "Rotor angular speed" annotation(
    Placement(transformation(extent = {{100, 30}, {120, 50}})));
  RealOutput i(unit = "A", start = km*w_start^2/kt) "Armature current" annotation(
    Placement(transformation(extent = {{100, -50}, {120, -30}})));

  SI.Torque tau_m "Electromagnetic torque";
  SI.Torque tau_p "Propeller drag torque";
  SI.Power P_el "Electrical power drawn";

initial equation
  if steadyStateInit then
    der(i) = 0;
    der(w) = 0;
  else
    i = 0;
    w = w_start;
  end if;

equation
  // Electrical part:  V = R i + L di/dt + ke w
  L*der(i) = V - R*i - ke*w;
  // Mechanical part:  Jm dw/dt = kt i - tau_p - bm w
  tau_m = kt*i;
  tau_p = km*w*abs(w) "= km w^2 for w >= 0";
  Jm*der(w) = tau_m - tau_p - bm*w;
  P_el = V*i;

  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Ellipse(lineColor = {0, 0, 127}, extent = {{-45, 45}, {45, -45}}), Text(textColor = {0, 0, 127}, extent = {{-40, 30}, {40, -30}}, textString = "M"), Text(textColor = {0, 0, 127}, extent = {{-95, 20}, {-50, -20}}, textString = "V"), Text(textColor = {0, 0, 127}, extent = {{50, 60}, {95, 20}}, textString = "ω"), Text(textColor = {0, 0, 127}, extent = {{50, -20}, {95, -60}}, textString = "i")}),
    Documentation(info = "<html><p>Equivalent DC model of one brushless motor + propeller (informe_1, eqs. 3-7):</p>
<p>L di/dt = V - R i - k<sub>e</sub> &omega;<br>J<sub>m</sub> d&omega;/dt = k<sub>t</sub> i - k<sub>m</sub> &omega;<sup>2</sup> - b<sub>m</sub> &omega;</p></html>"));
end ModeloMotor;
