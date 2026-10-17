within ModeloDrone.SubSistemas;

model ThrustTiltCompute

  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  
  parameter Real g=9.81;
  parameter Real mass;
  
  RealInput a[3](each unit = "m/s2") "Acceleration in" annotation(
    Placement(transformation(origin = {0, -76}, extent = {{-140, 55}, {-100, 95}}), iconTransformation(origin = {0, -70}, extent = {{-130, 55}, {-100, 85}})));
    
  RealOutput Fz(final quantity="Force", final unit = "N") "Force out" annotation(
    Placement(transformation(origin = {240, 2}, extent = {{-140, 55}, {-100, 95}}), iconTransformation(origin = {230, 4}, extent = {{-130, 55}, {-100, 85}})));
  
  RealOutput roll(final quantity = "Angle", final unit = "rad") "Roll out" annotation(
    Placement(transformation(origin = {240, -74}, extent = {{-140, 55}, {-100, 95}}), iconTransformation(origin = {230, -70}, extent = {{-130, 55}, {-100, 85}})));
    
  RealOutput pitch(final quantity = "Angle", final unit = "rad") "Pitch out" annotation(
    Placement(transformation(origin = {240, -112}, extent = {{-140, 55}, {-100, 95}}), iconTransformation(origin = {230, -110}, extent = {{-130, 55}, {-100, 85}})));
    
  RealOutput yaw(final quantity = "Angle", final unit = "rad") "Yaw out" annotation(
    Placement(transformation(origin = {240, -154}, extent = {{-140, 55}, {-100, 95}}), iconTransformation(origin = {230, -150}, extent = {{-130, 55}, {-100, 85}})));
  
  Real az_total;
  
equation
  
  az_total=a[3]-g;
  // Fz total es la suma de todas las componentes de aceleracion
  Fz = mass*sqrt(a[1]^2 + a[2]^2 + (az_total)^2);
  
  yaw = 0;
  // angulo entre ax/az_total
  pitch = atan2(a[1],az_total);
//
  roll = atan2(-a[2]*cos(pitch),az_total);
    
  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {134, 0},textColor = {0, 0, 127}, extent = {{-95, 95}, {-5, 55}}, textString = "F"), Text(origin = {-128, 4},textColor = {0, 0, 127}, extent = {{5, 20}, {95, -20}}, textString = "a")}),
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})));

end ThrustTiltCompute;