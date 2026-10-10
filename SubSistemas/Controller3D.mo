within ModeloDrone.SubSistemas;

block Controller3D "3-axis controller: one LimPID per axis"
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  import Modelica.Blocks.Types.SimpleController;
  
  parameter SimpleController controllerType = SimpleController.PID "P, PI, PD or PID";
  parameter Real k[3] = {1, 1, 1} "Gain per axis [x, y, z]";
  parameter Real Ti[3] = {1, 1, 1} "Integral time per axis";
  parameter Real Td[3] = {0.1, 0.1, 0.1} "Derivative time per axis";
  parameter Real yMax[3] = {10, 10, 10} "Output limit per axis";
  
  RealInput u_s[3] "Setpoint" annotation(
    Placement(transformation(origin = {0, -60}, extent = {{-140, 40}, {-100, 80}}), iconTransformation(origin = {0, -60}, extent = {{-140, 40}, {-100, 80}})));
  RealInput u_m[3] "Measurement" annotation(
    Placement(transformation(origin = {-60, 0}, extent = {{-140, -80}, {-100, -40}}, rotation = 90), iconTransformation(origin = {-60, 0}, extent = {{-140, -80}, {-100, -40}}, rotation = 90)));
  RealOutput y[3] "Control output" annotation(
    Placement(transformation(extent = {{100, -10}, {120, 10}})));
  Modelica.Blocks.Continuous.LimPID pid[3](each controllerType = controllerType, k = k, Ti = Ti, Td = Td, yMax = yMax);
  
equation
  for i in 1:3 loop
    connect(u_s[i], pid[i].u_s);
    connect(u_m[i], pid[i].u_m);
    connect(pid[i].y, y[i]);
  end for;
  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {100, -72},textColor = {0, 0, 127}, extent = {{-95, 95}, {-5, 55}}, textString = "y[3]"), Text(origin = {50, -100},textColor = {0, 0, 127}, extent = {{-95, 45}, {-5, 5}}, textString = "u_m[3]"), Text(origin = {-104, 4},textColor = {0, 0, 127}, extent = {{5, 20}, {95, -20}}, textString = "u_s[3]")}),
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})));
end Controller3D;