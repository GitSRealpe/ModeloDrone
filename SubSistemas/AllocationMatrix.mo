within ModeloDrone.SubSistemas;

block AllocationMatrix "Quadcopter allocation matrix"
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  import SI = Modelica.Units.SI;
  
  parameter SI.Length l = 0.2 "Propeller distance to CoG";
  parameter Real kf=1e-5  "Thrust coefficient";
  parameter Real km=1e-7  "Torque coefficient";
  
  RealInput w[4](each unit = "rad/s") "Rotor speeds input" annotation(
    Placement(transformation(origin = {0, -76}, extent = {{-140, 55}, {-100, 95}}), iconTransformation(origin = {0, -70}, extent = {{-130, 55}, {-100, 85}}))); 
  RealOutput F_z(final quantity="Force", final unit = "N") "Total force out" annotation(
    Placement(transformation(origin = {240, -16}, extent = {{-140, 55}, {-100, 95}}), iconTransformation(origin = {230, -10}, extent = {{-130, 55}, {-100, 85}})));
  RealOutput tau[3](each unit = "N.m") "Torques out" annotation(
    Placement(transformation(origin = {240, -116}, extent = {{-140, 55}, {-100, 95}}), iconTransformation(origin = {230, -130}, extent = {{-130, 55}, {-100, 85}})));
  
protected
  
  Real d=l/sqrt(2);
  Real k=km/kf;
  Real Ainv[4,4] = [1, -1/d,  1/d,  1/k;
                              1, -1/d, -1/d, -1/k;
                              1,  1/d, -1/d,  1/k;
                              1,  1/d,  1/d, -1/k];

  Real w2[4] "Squared rotor speeds";
equation
  w2 = 1/(4*kf)* Ainv * {F_z, tau_x, tau_y, tau_z};
  for i in 1:4 loop
    w[i] = min(sqrt(max(w2[i], 0)), w_max);
  end for;

  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(textColor = {0, 0, 127}, extent = {{-95, 95}, {-5, 55}}, textString = "Fz"), Text(textColor = {0, 0, 127}, extent = {{-95, 45}, {-5, 5}}, textString = "τx"), Text(textColor = {0, 0, 127}, extent = {{-95, -5}, {-5, -45}}, textString = "τy"), Text(textColor = {0, 0, 127}, extent = {{-95, -55}, {-5, -95}}, textString = "τz"), Text(textColor = {0, 0, 127}, extent = {{5, 20}, {95, -20}}, textString = "ω[4]")}),
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})));

end AllocationMatrix;