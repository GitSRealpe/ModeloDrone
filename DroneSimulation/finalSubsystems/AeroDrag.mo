within DroneSimulation.finalSubsystems;

model AeroDrag
  import SI = Modelica.Units.SI;
  parameter SI.Density rho = 1.225;
  parameter SI.Area CdA_h = 0.015 annotation(Dialog(group = "Drag"));
  parameter SI.Area CdA_v = 0.04 annotation(Dialog(group = "Drag"));

  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(extent = {{84, -16}, {116, 16}})));
  Modelica.Blocks.Interfaces.RealInput v_wind[3](each unit = "m/s") annotation(
    Placement(transformation(extent = {{-140, -20}, {-100, 20}})));

  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity velocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {70, 50}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Math.Feedback vRel[3] annotation(
    Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Abs absV[3] annotation(
    Placement(transformation(origin = {-30, -30}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Blocks.Math.Product vAbsV[3] annotation(
    Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Gain dragGain[3](k = 0.5*rho*{CdA_h, CdA_h, CdA_v}) annotation(
    Placement(transformation(origin = {30, 0}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Mechanics.MultiBody.Forces.WorldForce force(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world) annotation(
    Placement(transformation(origin = {66, 0}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(v_wind, vRel.u1) annotation(
    Line(points = {{-120, 0}, {-68, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(velocity.frame_a, frame_a) annotation(
    Line(points = {{80, 50}, {90, 50}, {90, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(velocity.v, vRel.u2) annotation(
    Line(points = {{59, 50}, {-60, 50}, {-60, 8}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vRel.y, absV.u) annotation(
    Line(points = {{-51, 0}, {-46, 0}, {-46, -30}, {-39.6, -30}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vRel.y, vAbsV.u1) annotation(
    Line(points = {{-51, 0}, {-20, 0}, {-20, 6}, {-12, 6}}, color = {0, 0, 127}, thickness = 0.5));
  connect(absV.y, vAbsV.u2) annotation(
    Line(points = {{-21.2, -30}, {-16, -30}, {-16, -6}, {-12, -6}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vAbsV.y, dragGain.u) annotation(
    Line(points = {{11, 0}, {20.4, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(dragGain.y, force.force) annotation(
    Line(points = {{38.8, 0}, {54, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(force.frame_b, frame_a) annotation(
    Line(points = {{76, 0}, {100, 0}}, color = {95, 95, 95}));
  annotation(
    Icon(graphics = {
      Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid),
      Line(points = {{-70, 20}, {40, 20}}, color = {0, 100, 200}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}),
      Line(points = {{-70, -20}, {40, -20}}, color = {0, 100, 200}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}),
      Text(extent = {{-100, 90}, {100, 50}}, textString = "drag"),
      Text(extent = {{-150, -105}, {150, -135}}, textString = "%name", textColor = {0, 0, 255})}));
end AeroDrag;
