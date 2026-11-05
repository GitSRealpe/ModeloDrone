within DroneSimulation.finalSubsystems;

model ESC
  Modelica.Electrical.Analog.Interfaces.PositivePin bat_p annotation(
    Placement(transformation(extent = {{-110, 50}, {-90, 70}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin bat_n annotation(
    Placement(transformation(extent = {{-110, -70}, {-90, -50}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin mot_p annotation(
    Placement(transformation(extent = {{90, 50}, {110, 70}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin mot_n annotation(
    Placement(transformation(extent = {{90, -70}, {110, -50}})));
  Modelica.Blocks.Interfaces.RealInput duty annotation(
    Placement(transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));

  Modelica.Blocks.Nonlinear.Limiter limiter(uMax = 1, uMin = 0) annotation(
    Placement(transformation(origin = {0, 84}, extent = {{-6, -6}, {6, 6}}, rotation = 270)));
  Modelica.Electrical.Analog.Sensors.VoltageSensor vBat annotation(
    Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
  Modelica.Electrical.Analog.Sources.SignalCurrent iBat annotation(
    Placement(transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
  Modelica.Blocks.Math.Product iBatCalc annotation(
    Placement(transformation(origin = {-6, 30}, extent = {{6, -6}, {-6, 6}})));
  Modelica.Electrical.Analog.Sources.SignalVoltage vMot annotation(
    Placement(transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Math.Product vMotCalc annotation(
    Placement(transformation(origin = {10, 0}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Electrical.Analog.Sensors.CurrentSensor iMot annotation(
    Placement(transformation(origin = {70, 60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Electrical.Analog.Basic.Ground groundMotor annotation(
    Placement(transformation(origin = {40, -80}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(duty, limiter.u) annotation(
    Line(points = {{0, 120}, {0, 91}}, color = {0, 0, 127}));
  connect(bat_p, vBat.p) annotation(
    Line(points = {{-100, 60}, {-70, 60}, {-70, 10}}, color = {0, 0, 255}));
  connect(bat_n, vBat.n) annotation(
    Line(points = {{-100, -60}, {-70, -60}, {-70, -10}}, color = {0, 0, 255}));
  connect(bat_p, iBat.p) annotation(
    Line(points = {{-100, 60}, {-30, 60}, {-30, 10}}, color = {0, 0, 255}));
  connect(bat_n, iBat.n) annotation(
    Line(points = {{-100, -60}, {-30, -60}, {-30, -10}}, color = {0, 0, 255}));
  connect(limiter.y, iBatCalc.u1) annotation(
    Line(points = {{0, 77.4}, {0, 40}, {8, 40}, {8, 33.6}, {1.2, 33.6}}, color = {0, 0, 127}));
  connect(iMot.i, iBatCalc.u2) annotation(
    Line(points = {{70, 49}, {70, 26.4}, {1.2, 26.4}}, color = {0, 0, 127}));
  connect(iBatCalc.y, iBat.i) annotation(
    Line(points = {{-12.6, 30}, {-18, 30}, {-18, 0}, {-18, 0}}, color = {0, 0, 127}));
  connect(limiter.y, vMotCalc.u1) annotation(
    Line(points = {{0, 77.4}, {0, 3.6}, {2.8, 3.6}}, color = {0, 0, 127}));
  connect(vBat.v, vMotCalc.u2) annotation(
    Line(points = {{-59, 0}, {-50, 0}, {-50, -20}, {-6, -20}, {-6, -3.6}, {2.8, -3.6}}, color = {0, 0, 127}));
  connect(vMotCalc.y, vMot.v) annotation(
    Line(points = {{16.6, 0}, {28, 0}}, color = {0, 0, 127}));
  connect(vMot.p, iMot.p) annotation(
    Line(points = {{40, 10}, {40, 60}, {60, 60}}, color = {0, 0, 255}));
  connect(iMot.n, mot_p) annotation(
    Line(points = {{80, 60}, {100, 60}}, color = {0, 0, 255}));
  connect(vMot.n, mot_n) annotation(
    Line(points = {{40, -10}, {40, -60}, {100, -60}}, color = {0, 0, 255}));
  connect(vMot.n, groundMotor.p) annotation(
    Line(points = {{40, -10}, {40, -70}}, color = {0, 0, 255}));
  annotation(
    Icon(graphics = {
      Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid),
      Text(extent = {{-90, 40}, {90, -40}}, textString = "ESC"),
      Text(extent = {{-95, 90}, {-30, 70}}, textString = "bat"),
      Text(extent = {{30, 90}, {95, 70}}, textString = "mot"),
      Text(extent = {{-150, -105}, {150, -135}}, textString = "%name", textColor = {0, 0, 255})}));
end ESC;
