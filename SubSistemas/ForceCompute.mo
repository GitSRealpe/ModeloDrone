within ModeloDrone.SubSistemas;

model ForceCompute
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  parameter Real g = Modelica.Constants.g_n;
  parameter Real mass;
  RealInput a_cmd[3](each unit = "m/s2") "Commanded acceleration" annotation(
    Placement(transformation(origin = {0, -76}, extent = {{-140, 55}, {-100, 95}}), iconTransformation(origin = {0, -70}, extent = {{-130, 55}, {-100, 85}})));
  RealInput R_m[3, 3] "Measured rotation, body -> world" annotation(
    Placement(transformation(origin = {-60, 0}, extent = {{-140, -80}, {-100, -40}}, rotation = 90), iconTransformation(origin = {-60, 0}, extent = {{-140, -80}, {-100, -40}}, rotation = 90)));
  RealOutput T(unit = "N") "Total thrust along body z" annotation(
    Placement(transformation(origin = {240, -76}, extent = {{-140, 55}, {-100, 95}}), iconTransformation(origin = {230, -72}, extent = {{-130, 55}, {-100, 85}})));
protected
  Real F_des[3] = mass*{a_cmd[1], a_cmd[2], a_cmd[3] + g};
public
equation
  T = max(0, F_des[1]*R_m[1, 3] + F_des[2]*R_m[2, 3] + F_des[3]*R_m[3, 3]);
  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {128, -74}, textColor = {0, 0, 127}, extent = {{-95, 95}, {-5, 55}}, textString = "T"), Text(origin = {-118, 4}, textColor = {0, 0, 127}, extent = {{5, 20}, {95, -20}}, textString = "a[3]"), Text(origin = {-56, -76}, textColor = {0, 0, 127}, extent = {{5, 20}, {95, -20}}, textString = "R[3,3]")}),
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})));
end ForceCompute;