within ModeloDrone.SubSistemas;

model Perturbaciones "External forces in the inertial frame: aerodynamic drag + wind"
  import SI = Modelica.Units.SI;
  import Modelica.Mechanics.MultiBody.Frames;

  parameter Real c_d(unit = "N.s/m") = 0 "Linear drag coefficient (0 = no drag)";
  parameter SI.Force F_wind[3] = {0, 0, 0} "Wind force, world frame";
  parameter SI.Time t_wind = 0 "Time at which the wind starts";

  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b "Connect to the body" annotation(
    Placement(transformation(extent = {{84, -16}, {116, 16}})));

  SI.Velocity v[3] "Body velocity, world frame";
  SI.Force F_d[3] "Drag force, world frame";
  SI.Force F_w[3] "Wind force, world frame";
  SI.Force F[3] "Total disturbance force, world frame";

equation
  v = der(frame_b.r_0);
  F_d = -c_d*v;
  F_w = if time >= t_wind then F_wind else zeros(3);
  F = F_d + F_w;
  // Applied like MultiBody.Forces.WorldForce (resolved in world frame)
  frame_b.f = -Frames.resolve2(frame_b.R, F);
  frame_b.t = zeros(3);

  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Line(points = {{-80, 40}, {40, 40}}, color = {0, 0, 127}, arrow = {Arrow.None, Arrow.Filled}), Line(points = {{-80, 0}, {60, 0}}, color = {0, 0, 127}, arrow = {Arrow.None, Arrow.Filled}), Line(points = {{-80, -40}, {40, -40}}, color = {0, 0, 127}, arrow = {Arrow.None, Arrow.Filled}), Text(textColor = {0, 0, 127}, extent = {{-90, -55}, {90, -90}}, textString = "Fd + Fwind")}),
    Documentation(info = "<html><p>Disturbance forces from eq. (8) of Modelado_Trabajo: F = F<sub>T</sub> + F<sub>g</sub> + F<sub>d</sub>. Here F<sub>d</sub> = -c<sub>d</sub> v, plus the wind force of eq. (40), F<sub>wind</sub> = [F<sub>x</sub> 0 0]<sup>T</sup>. Both are expressed in the inertial (world) frame.</p></html>"));
end Perturbaciones;
