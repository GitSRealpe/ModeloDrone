within DroneSimulation.finalSubsystems;

model Sensors
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin bat_p annotation(
    Placement(transformation(origin = {-40, -100}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin bat_n annotation(
    Placement(transformation(origin = {40, -100}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealOutput r[3](each unit = "m") annotation(
    Placement(transformation(origin = {-60, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealOutput v[3](each unit = "m/s") annotation(
    Placement(transformation(origin = {-20, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealOutput angles[3](each unit = "rad") annotation(
    Placement(transformation(origin = {20, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealOutput v_bat(unit = "V") annotation(
    Placement(transformation(origin = {60, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {-60, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {-20, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles annotation(
    Placement(transformation(origin = {20, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor annotation(
    Placement(transformation(origin = {60, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
equation
  connect(frame_a, absolutePosition.frame_a) annotation(
    Line(points = {{-100, 0}, {-60, 0}, {-60, 40}}, color = {95, 95, 95}));
  connect(frame_a, absoluteVelocity.frame_a) annotation(
    Line(points = {{-100, 0}, {-20, 0}, {-20, 40}}, color = {95, 95, 95}));
  connect(frame_a, absoluteAngles.frame_a) annotation(
    Line(points = {{-100, 0}, {20, 0}, {20, 40}}, color = {95, 95, 95}));
  connect(absolutePosition.r, r) annotation(
    Line(points = {{-60, 61}, {-60, 110}}, color = {0, 0, 127}, thickness = 0.5));
  connect(absoluteVelocity.v, v) annotation(
    Line(points = {{-20, 61}, {-20, 110}}, color = {0, 0, 127}, thickness = 0.5));
  connect(absoluteAngles.angles, angles) annotation(
    Line(points = {{20, 61}, {20, 110}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bat_p, voltageSensor.p) annotation(
    Line(points = {{-40, -100}, {-40, -70}, {60, -70}, {60, -60}}, color = {0, 0, 255}));
  connect(bat_n, voltageSensor.n) annotation(
    Line(points = {{40, -100}, {40, -80}, {80, -80}, {80, -30}, {60, -30}, {60, -40}}, color = {0, 0, 255}));
  connect(voltageSensor.v, v_bat) annotation(
    Line(points = {{71, -50}, {90, -50}, {90, 90}, {60, 90}, {60, 110}}, color = {0, 0, 127}));
  annotation(
    Icon(graphics = {
      Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {150, 100, 0}, fillColor = {255, 250, 235}, fillPattern = FillPattern.Solid, radius = 10),
      Bitmap(extent = {{-90, 90}, {90, -90}}, fileName = "modelica://DroneSimulation/finalSubsystems/Resources/icon_feedback.png"),
      Text(extent = {{-150, -105}, {150, -140}}, textString = "%name", textColor = {150, 100, 0})}));
end Sensors;
