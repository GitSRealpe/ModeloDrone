within ModeloDrone.SubSistemas;

block AttitudeController
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  import Modelica.Blocks.Types.SimpleController;
  
  // inertia values
  parameter Real Ixx;
  parameter Real Iyy;
  parameter Real Izz;
  
  parameter Real kR = 100 "1/s^2";
  parameter Real kW = 20  "1/s";

  RealInput u_s[3] "Desired angles {roll, pitch, yaw} in rad" annotation(
      Placement(transformation(origin = {0, -60}, extent = {{-140, 40}, {-100, 80}}), iconTransformation(origin = {0, -60}, extent = {{-140, 40}, {-100, 80}}))); 
  RealInput w_m[3] "Measured angular velocity in body frame" annotation(
    Placement(transformation( extent = {{-140, -80}, {-100, -40}}, rotation = 90), iconTransformation(origin = {6, 0}, extent = {{-140, -80}, {-100, -40}}, rotation = 90)));
  RealInput R_m[3,3] "Measured rotation in body frame" annotation(
    Placement(transformation(origin = {-120, 0}, extent = {{-140, -80}, {-100, -40}}, rotation = 90), iconTransformation(origin = {-126, 0}, extent = {{-140, -80}, {-100, -40}}, rotation = 90)));
  
  RealOutput y[3] "Control output" annotation(
      Placement(transformation(extent = {{100, -10}, {120, 10}})));
      
protected
  Real phi   = u_s[1];
  Real theta = u_s[2];
  Real psi   = u_s[3];
  Real R_d[3,3] "Desired rotation matrix R*, body -> world";
  Real J[3,3] = diagonal({Ixx, Iyy, Izz});
  Real S[3,3], eR[3], eW[3];
  
public
equation

  R_d = [cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta),  -cos(phi)*sin(psi),  cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi);
        cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),   cos(phi)*cos(psi),  sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi);
        -cos(phi)*sin(theta),                                 sin(phi),           cos(phi)*cos(theta)];
        
  S  = transpose(R_m)*R_d - transpose(R_d)*R_m;
  eR = 0.5*{S[3,2], S[1,3], S[2,1]};
  eW = w_m;                                  // Omega* = 0
  y  = J*(kR*eR - kW*eW) + cross(w_m, J*w_m);
        
        
  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {112, -72}, textColor = {0, 0, 127}, extent = {{-95, 95}, {-5, 55}}, textString = "y[3]"), Text(origin = {-104, 4}, textColor = {0, 0, 127}, extent = {{5, 20}, {95, -20}}, textString = "u_s[3]"), Text(origin = {-17, -101}, textColor = {0, 0, 127}, extent = {{-78, 30}, {-4, 3}}, textString = "R_m[3,3]"), Text(origin = {101, -101}, textColor = {0, 0, 127}, extent = {{-78, 30}, {-4, 3}}, textString = "w_m[3]")}),
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})));

end AttitudeController;