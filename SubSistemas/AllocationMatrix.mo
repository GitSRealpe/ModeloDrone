within ModeloDrone.SubSistemas;

block AllocationMatrix "Quadcopter allocation matrix"
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  import SI = Modelica.Units.SI;
  
  parameter SI.Length l "Propeller distance to CoG";
  parameter Real kf     "Thrust coefficient";
  parameter Real km     "Torque coefficient";
  
  RealInput w[4](each unit = "rad/s") "Rotor speeds input" annotation(
    Placement(transformation(origin = {0, -76}, extent = {{-140, 55}, {-100, 95}}), iconTransformation(origin = {0, -70}, extent = {{-130, 55}, {-100, 85}}))); 
  RealOutput F_z(final quantity="Force", final unit = "N") "Total force out" annotation(
    Placement(transformation(origin = {240, -16}, extent = {{-140, 55}, {-100, 95}}), iconTransformation(origin = {230, -10}, extent = {{-130, 55}, {-100, 85}})));
  RealOutput tau[3](each unit = "N.m") "Torques out" annotation(
    Placement(transformation(origin = {240, -116}, extent = {{-140, 55}, {-100, 95}}), iconTransformation(origin = {230, -130}, extent = {{-130, 55}, {-100, 85}})));
  
protected
  
  parameter Real d=l/sqrt(2);
  parameter Real k=km/kf;
  parameter Real A[4,4] = [1, 1,  1,  1;
                           -d, -d, d, d;
                            d,  -d, -d, d;
                            k,  -k,  k, -k];

equation
  
  {F_z, tau[1], tau[2], tau[3]} =  A * (kf * w.^2);  

  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {126, -12},textColor = {0, 0, 127}, extent = {{-95, 95}, {-5, 55}}, textString = "Fz"), Text(origin = {120, -82},textColor = {0, 0, 127}, extent = {{-95, 45}, {-5, 5}}, textString = "τ[3]"), Text(origin = {-114, 4},textColor = {0, 0, 127}, extent = {{5, 20}, {95, -20}}, textString = "ω[4]")}),
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})));

end AllocationMatrix;
