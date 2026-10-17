within ModeloDrone.SubSistemas;

model ForceCompute

  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  
  parameter Real g=Modelica.Constants.g_n;
  parameter Real mass;
  
  RealInput a(unit = "m/s2") "Acceleration in" annotation(
    Placement(transformation(origin = {0, -76}, extent = {{-140, 55}, {-100, 95}}), iconTransformation(origin = {0, -70}, extent = {{-130, 55}, {-100, 85}})));
    
  RealOutput F(final quantity="Force", final unit = "N") "Force out" annotation(
    Placement(transformation(origin = {240, -76}, extent = {{-140, 55}, {-100, 95}}), iconTransformation(origin = {230, -72}, extent = {{-130, 55}, {-100, 85}})));
  
equation

  F = mass*(a+g);
  
    annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {128, -74},textColor = {0, 0, 127}, extent = {{-95, 95}, {-5, 55}}, textString = "F"), Text(origin = {-128, 4},textColor = {0, 0, 127}, extent = {{5, 20}, {95, -20}}, textString = "a")}),
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})));

end ForceCompute;