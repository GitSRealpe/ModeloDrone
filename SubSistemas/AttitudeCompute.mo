within ModeloDrone.SubSistemas;

model AttitudeCompute
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  
  parameter Real g=9.81;
  
  RealInput a_xy[3](each unit = "m/s2") "Acceleration in" annotation(
    Placement(transformation(origin = {0, -14}, extent = {{-140, 55}, {-100, 95}}), iconTransformation(origin = {0, -10}, extent = {{-130, 55}, {-100, 85}})));
    
  RealInput yaw_d(unit = "rad") "Yaw in" annotation(
    Placement(transformation(origin = {0, -136}, extent = {{-140, 55}, {-100, 95}}), iconTransformation(origin = {0, -130}, extent = {{-130, 55}, {-100, 85}})));
  
  RealOutput rpy_d[3](each unit = "rad") "RPY out" annotation(
    Placement(transformation(origin = {240, -74}, extent = {{-140, 55}, {-100, 95}}), iconTransformation(origin = {230, -70}, extent = {{-130, 55}, {-100, 85}})));
  
  parameter Real theta_max(unit = "rad" ) = 0.4 "max tilt";

protected
  Real a_n = sqrt(a_xy[1]^2 + a_xy[2]^2);
  Real k   = if a_n > g*tan(theta_max) then g*tan(theta_max)/a_n else 1;
  
equation
  
  // roll
  rpy_d[1] = k*(a_xy[1]*sin(yaw_d) - a_xy[2]*cos(yaw_d))/g;
  // pitch
  rpy_d[2] = k*(a_xy[1]*cos(yaw_d) + a_xy[2]*sin(yaw_d))/g;
  //yaw_d
  rpy_d[3] = yaw_d;
    
  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {110, -70}, textColor = {0, 0, 127}, extent = {{-95, 95}, {-5, 55}}, textString = "rpy[3]"), Text(origin = {-101, 64},textColor = {0, 0, 127}, extent = {{4, 18}, {82, -18}}, textString = "a_xy[2]"), Text(origin = {-101, -54}, textColor = {0, 0, 127}, extent = {{4, 18}, {82, -18}}, textString = "yaw_d")}),
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})));

end AttitudeCompute;