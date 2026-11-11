within ModeloDrone.SubSistemas;

model OrientationSensor "Outputs R (body->world) and Omega (body frame)"
  import Modelica.Mechanics.MultiBody.Interfaces.Frame_a;

  Frame_a frame_a "Connect to the body's frame_a" annotation(
      Placement(transformation(origin = {0, -60}, extent = {{-140, 40}, {-100, 80}}), iconTransformation(origin = {0, -60}, extent = {{-140, 40}, {-100, 80}})));
      
  Modelica.Blocks.Interfaces.RealOutput R[3,3] "Rotation matrix, body -> world" annotation(
      Placement(transformation(origin = {0, 60}, extent = {{100, -10}, {120, 10}}), iconTransformation(origin = {0, 60}, extent = {{100, -10}, {120, 10}})));
  Modelica.Blocks.Interfaces.RealOutput Omega[3](each unit = "rad/s") "Angular velocity in body frame" annotation(
      Placement(transformation(origin = {0, -60}, extent = {{100, -10}, {120, 10}}), iconTransformation(origin = {0, -60}, extent = {{100, -10}, {120, 10}})));

equation
  R = transpose(frame_a.R.T);
  Omega = frame_a.R.w;
  // A sensor must not load the frame
  frame_a.f = zeros(3);
  frame_a.t = zeros(3);
  
  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {106, -10},textColor = {0, 0, 127}, extent = {{-95, 95}, {-5, 55}}, textString = "R[3,3]"), Text(origin = {114, -82},textColor = {0, 0, 127}, extent = {{-95, 45}, {-5, 5}}, textString = "w[3]"), Text(origin = {-104, 4},textColor = {0, 0, 127}, extent = {{5, 20}, {95, -20}}, textString = "frame")}),
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Line(origin = {-108, 0}, points = {{-10, 0}, {8, 0}})}));
  
end OrientationSensor;