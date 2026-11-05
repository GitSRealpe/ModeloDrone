within DroneLibrary.Mechanical.Chassis.Variants;
model droneChassis
  extends DroneLibrary.Mechanical.Chassis.Templates.droneChassis(bodyShape(animation = false), bodyShape1(animation = false), bodyShape2(animation = false), bodyShape3(animation = false), bodyCylinder(animation = false), pointMass(animation = false));

  Modelica.Mechanics.MultiBody.Visualizers.FixedShape fixedShape(
    animation=false,
    shapeType="modelica://DroneLibrary/Resources/Images/9.stl",
    r_shape={0,0,0},
    lengthDirection = {1,0,0},
    length=1.025e-3,
    width=1.025e-3,
    height=1.025e-3,
    extra=1) annotation (Placement(transformation(origin = {40, 4}, extent = {{88, 20}, {108, 40}})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRotation(
    animation= false,
    r={-0.27,0,-0.2},
    angle=90) annotation (Placement(transformation(origin = {-5, -10}, extent = {{45, 30}, {75, 60}})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRotation1(
    animation= false,
    n = {0,1,0},
    angle=45) annotation (Placement(transformation(origin = {28, 4}, extent = {{58, 20}, {78, 40}})));
  parameter Boolean animation=true "= true, if animation shall be enabled";
equation
  connect(fixedRotation.frame_b,fixedRotation1. frame_a) annotation (Line(
      points={{70, 35}, {70, 34},{86,34}},
      color={95,95,95},
      thickness=0.5));
  connect(fixedShape.frame_a,fixedRotation1. frame_b) annotation (Line(
      points={{128, 34}, {106, 34}},
      color={95,95,95},
      thickness=0.5));
  connect(fixedRotation.frame_a, bodyCylinder.frame_a) annotation (Line(
      points={{40,35},{16,35},{16,0},{32,0}},
      color={95,95,95},
      thickness=0.5));
annotation(
    experiment(StartTime = 0, StopTime = 50, Tolerance = 1e-06, Interval = 0.01));
end droneChassis;