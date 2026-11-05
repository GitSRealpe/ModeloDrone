within DroneLibrary.Tests;

model motorTest
  extends Modelica.Icons.Example;
  DroneLibrary.Electrical.Machines.SimpleDCmotor dCmotor annotation(
    Placement(transformation(origin = {36, 38}, extent = {{-34, -4}, {34, 36}})));
  Modelica.Blocks.Sources.Step step(startTime = 1) annotation(
    Placement(transformation(origin = {-72, 32}, extent = {{-86, -2}, {-66, 18}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
    Placement(transformation(origin = {90, -38}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Math.Feedback feedback annotation(
    Placement(transformation(extent = {{-58, -2}, {-38, 18}})));
  Modelica.Blocks.Sources.Step step1(startTime = 2) annotation(
    Placement(transformation(origin = {-82, 8}, extent = {{-76, -46}, {-56, -26}})));
  Mechanical.Propeller.Variants.Propeller propellerRev1 annotation(
    Placement(transformation(origin = {20, 28}, extent = {{-30, -114}, {30, -84}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed1(r = {0.5, 0, 0}) annotation(
    Placement(transformation(origin = {110, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Mechanical.Propeller.Variants.Propeller propellerRev2 annotation(
    Placement(transformation(origin = {21, 14}, extent = {{-29, -44}, {29, -24}})));
equation
  connect(step.y, feedback.u1) annotation(
    Line(points = {{-137, 40}, {-60.5, 40}, {-60.5, 8}, {-56, 8}}, color = {0, 0, 127}));
  connect(feedback.y, dCmotor.current) annotation(
    Line(points = {{-39, 8}, {-25.5, 8}, {-25.5, 54}, {-5, 54}}, color = {0, 0, 127}));
  connect(step1.y, feedback.u2) annotation(
    Line(points = {{-137, -28}, {-50, -28}, {-50, 0}, {-48, 0}}, color = {0, 0, 127}));
  connect(propellerRev1.motorCommand, dCmotor.current) annotation(
    Line(points = {{-17, -74}, {-24, -74}, {-24, 54}, {-5, 54}}, color = {0, 0, 127}));
  connect(propellerRev2.motorCommand, dCmotor.current) annotation(
    Line(points = {{-14, -22}, {-24, -22}, {-24, 54}, {-5, 54}}, color = {0, 0, 127}));
  connect(fixed.frame_b, propellerRev2.Airframe) annotation(
    Line(points = {{80, -38}, {16, -38}, {16, -24}, {51, -24}}, color = {95, 95, 95}, thickness = 0.5));
  connect(fixed1.frame_b, propellerRev1.Airframe) annotation(
    Line(points = {{100, -74}, {76.5, -74}, {76.5, -77}, {51, -77}}, color = {95, 95, 95}, thickness = 0.5));
  annotation(
    experiment(StopTime = 15));
end motorTest;