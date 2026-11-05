within DroneSimulation.finalSubsystems;

block MotorMixerVoltage
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.Blocks.Interfaces.RealOutput;
  import SI = Modelica.Units.SI;

  parameter SI.Length d;
  parameter Real kf(unit = "N.s2/rad2");
  parameter Real km(unit = "N.m.s2/rad2");
  parameter Real psi(unit = "V.s/rad");
  parameter SI.Resistance Ra;
  final parameter Real c(unit = "m") = km/kf;

  RealInput F_z(final quantity = "Force", final unit = "N") annotation(
    Placement(transformation(extent = {{-140, 55}, {-100, 95}}), iconTransformation(extent = {{-130, 55}, {-100, 85}})));
  RealInput tau_x(unit = "N.m") annotation(
    Placement(transformation(extent = {{-140, 5}, {-100, 45}}), iconTransformation(extent = {{-130, 5}, {-100, 35}})));
  RealInput tau_y(unit = "N.m") annotation(
    Placement(transformation(extent = {{-140, -45}, {-100, -5}}), iconTransformation(extent = {{-130, -45}, {-100, -15}})));
  RealInput tau_z(unit = "N.m") annotation(
    Placement(transformation(extent = {{-140, -95}, {-100, -55}}), iconTransformation(extent = {{-130, -95}, {-100, -65}})));
  RealInput v_bat(unit = "V") annotation(
    Placement(transformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {0, -115}, extent = {{-15, -15}, {15, 15}}, rotation = 90)));
  RealOutput duty[4] annotation(
    Placement(transformation(extent = {{100, -10}, {120, 10}}), iconTransformation(extent = {{100, -10}, {120, 10}})));

  SI.Force T[4];
  SI.AngularVelocity w[4];
  SI.Voltage V[4];
equation
  T[1] = F_z/4 - tau_y/(2*d) - tau_z/(4*c);
  T[2] = F_z/4 + tau_x/(2*d) + tau_z/(4*c);
  T[3] = F_z/4 + tau_y/(2*d) - tau_z/(4*c);
  T[4] = F_z/4 - tau_x/(2*d) + tau_z/(4*c);
  for j in 1:4 loop
    w[j] = sqrt(max(T[j], 0)/kf);
    V[j] = psi*w[j] + Ra*km*w[j]^2/psi;
    duty[j] = V[j]/max(v_bat, 0.1);
  end for;
  annotation(
    Icon(graphics = {
      Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid),
      Text(extent = {{-90, 40}, {90, -40}}, textString = "Mixer V"),
      Text(extent = {{-95, 85}, {-50, 65}}, textString = "Fz"),
      Text(extent = {{-95, 30}, {-50, 10}}, textString = "τx"),
      Text(extent = {{-95, -20}, {-50, -40}}, textString = "τy"),
      Text(extent = {{-95, -70}, {-50, -90}}, textString = "τz"),
      Text(extent = {{-30, -70}, {30, -95}}, textString = "Vbat"),
      Text(extent = {{40, 20}, {95, -20}}, textString = "d"),
      Text(extent = {{-150, 140}, {150, 110}}, textString = "%name", textColor = {0, 0, 255})}));
end MotorMixerVoltage;
