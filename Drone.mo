within ModeloDrone;

model Drone

  import SI = Modelica.Units.SI;
  parameter SI.Length l = 0.2 "Propeller distance to CoG";
  parameter Real kf = 1e-5 "Thrust coefficient";
  parameter Real km = 1e-7 "Torque coefficient";
  parameter Real w_max(unit = "rad/s") = 1000 "Max rotor speed";
  parameter SI.Mass mass = 0.2 "Drone mass";
  parameter Real Ixx = 0.0082, Iyy = 0.0082, Izz = 0.0149 "Drone inertia values";

  SubSistemas.AllocationMatrix allocMat(l=l, kf=kf, km=km) annotation(
    Placement(transformation(origin = {38, 0}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.invAllocationMatrix invAllocMat(l=l, kf=kf, km=km, w_max=w_max) annotation(
    Placement(transformation(origin = {-12, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body body(m = mass, I_11 = Ixx, I_22 = Iyy, I_33 = Izz, angles_start (each displayUnit = "rad"), r_CM = {0, 0, 0})  annotation(
    Placement(transformation(origin = {174, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Forces.ForceAndTorque forceAndTorque(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_b)  annotation(
    Placement(transformation(origin = {126, -30}, extent = {{-10, -10}, {10, 10}})));
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
    Placement(transformation(origin = {44, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Routing.Multiplex3 multiplex3 annotation(
    Placement(transformation(origin = {96, 14}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const13(k = 0) annotation(
    Placement(transformation(origin = {38, 32}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const14(k = 0) annotation(
    Placement(transformation(origin = {38, 62}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.Controller3D vel_controller3D(controllerType = Modelica.Blocks.Types.SimpleController.PI) annotation(
    Placement(transformation(origin = {-179, 29}, extent = {{-15, -15}, {15, 15}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world)  annotation(
    Placement(transformation(origin = {-10, -60}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  SubSistemas.ForceCompute forceCompute(mass=mass) annotation(
    Placement(transformation(origin = {-120, 28}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant testz(k = 1)  annotation(
    Placement(transformation(origin = {-376, 82}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant testx(k = 0) annotation(
    Placement(transformation(origin = {-322, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Sources.Constant testy(k = 0) annotation(
    Placement(transformation(origin = {-334, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Routing.Multiplex3 multiplex4 annotation(
    Placement(transformation(origin = {-358, 42}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world)  annotation(
    Placement(transformation(origin = {-50, -164}, extent = {{10, -10}, {-10, 10}})));
  ModeloDrone.SubSistemas.Controller3D pos_controller3D(controllerType = Modelica.Blocks.Types.SimpleController.P) annotation(
    Placement(transformation(origin = {-251, 27}, extent = {{-15, -15}, {15, 15}})));
  SubSistemas.AttitudeController attitudeController(Ixx=Ixx, Iyy=Iyy, Izz=Izz) annotation(
    Placement(transformation(origin = {-205, -49}, extent = {{-15, -15}, {15, 15}})));
  SubSistemas.OrientationSensor orientationSensor annotation(
    Placement(transformation(origin = {-95, -105}, extent = {{13, -13}, {-13, 13}}, rotation = -0)));
  Modelica.Blocks.Sources.Constant testz1(k = 0) annotation(
    Placement(transformation(origin = {-330, -92}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant testx1(k = 0) annotation(
    Placement(transformation(origin = {-338, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Sources.Constant testy1(k = 0.1) annotation(
    Placement(transformation(origin = {-358, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Routing.Multiplex3 multiplex41 annotation(
    Placement(transformation(origin = {-268, -62}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Routing.DeMultiplex3 deMultiplex3 annotation(
  SubSistemas.AttitudeCompute attitudeCompute annotation(
    Placement(transformation(origin = {-108, -66}, extent = {{-18, -18}, {18, 18}})));
equation
  connect(invAllocMat.w, allocMat.w) annotation(
    Line(points = {{0, 0}, {26, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(allocMat.tau, forceAndTorque.torque) annotation(
    Line(points = {{50, -6}, {126, -6}, {126, -18}}, color = {0, 0, 127}, thickness = 0.5));
  connect(world.frame_b, forceAndTorque.frame_a) annotation(
    Line(points = {{54, -30}, {116, -30}}, color = {95, 95, 95}));
  connect(forceAndTorque.frame_b, body.frame_a) annotation(
    Line(points = {{136, -30}, {164, -30}}, color = {95, 95, 95}));
  connect(allocMat.F_z, multiplex3.u3[1]) annotation(
    Line(points = {{50, 6}, {65, 6}, {65, 7}, {84, 7}}, color = {0, 0, 127}));
  connect(const13.y, multiplex3.u2[1]) annotation(
    Line(points = {{49, 32}, {60, 32}, {60, 14}, {84, 14}}, color = {0, 0, 127}));
  connect(const14.y, multiplex3.u1[1]) annotation(
    Line(points = {{50, 62}, {68, 62}, {68, 22}, {84, 22}}, color = {0, 0, 127}));
  connect(multiplex3.y, forceAndTorque.force) annotation(
    Line(points = {{108, 14}, {118, 14}, {118, -18}}, color = {0, 0, 127}, thickness = 0.5));
  connect(absoluteVelocity.frame_a, body.frame_a) annotation(
    Line(points = {{0, -60}, {164, -60}, {164, -30}}, color = {95, 95, 95}));
  connect(forceCompute.F, invAllocMat.F_z) annotation(
    Line(points = {{-108.5, 28}, {-40, 28}, {-40, 8}, {-24, 8}}, color = {0, 0, 127}));
  connect(testz.y, multiplex4.u3[1]) annotation(
    Line(points = {{-376, 71}, {-376, 53}, {-365, 53}}, color = {0, 0, 127}));
  connect(testy.y, multiplex4.u2[1]) annotation(
    Line(points = {{-345, 84}, {-345, 86}, {-359, 86}, {-359, 54}}, color = {0, 0, 127}));
  connect(testx.y, multiplex4.u1[1]) annotation(
    Line(points = {{-333, 54}, {-352, 54}}, color = {0, 0, 127}));
  connect(vel_controller3D.y[3], forceCompute.a) annotation(
    Line(points = {{-162.5, 29}, {-141, 29}, {-141, 28}, {-131.5, 28}}, color = {0, 0, 127}));
  connect(absoluteVelocity.v, vel_controller3D.u_m) annotation(
    Line(points = {{-20, -60}, {-167, -60}, {-167, 11}, {-179, 11}}, color = {0, 0, 127}, thickness = 0.5));
  connect(absolutePosition.frame_a, body.frame_a) annotation(
    Line(points = {{-40, -164}, {164, -164}, {164, -30}}, color = {95, 95, 95}));
  connect(pos_controller3D.y, vel_controller3D.u_s) annotation(
    Line(points = {{-234, 28}, {-197, 28}, {-197, 29}}, color = {0, 0, 127}, thickness = 0.5));
  connect(absolutePosition.r, pos_controller3D.u_m) annotation(
    Line(points = {{-61, -164}, {-250, -164}, {-250, 10}}, color = {0, 0, 127}, thickness = 0.5));
  connect(multiplex4.y, pos_controller3D.u_s) annotation(
    Line(points = {{-358, 32}, {-358, 28}, {-268, 28}}, color = {0, 0, 127}, thickness = 0.5));
  connect(orientationSensor.frame_a, body.frame_a) annotation(
    Line(points = {{-79, -105}, {164, -105}, {164, -30}}, color = {95, 95, 95}));
  connect(orientationSensor.Omega, attitudeController.w_m) annotation(
    Line(points = {{-110, -112}, {-196, -112}, {-196, -66}}, color = {0, 0, 127}, thickness = 0.5));
  connect(orientationSensor.R, attitudeController.R_m) annotation(
    Line(points = {{-110, -98}, {-214, -98}, {-214, -66}}, color = {0, 0, 127}, thickness = 0.5));
  connect(testz1.y, multiplex41.u3[1]) annotation(
    Line(points = {{-319, -92}, {-319, -69}, {-280, -69}}, color = {0, 0, 127}));
  connect(testy1.y, multiplex41.u2[1]) annotation(
    Line(points = {{-358, -75}, {-358, -64.5}, {-280, -64.5}, {-280, -62}}, color = {0, 0, 127}));
  connect(testx1.y, multiplex41.u1[1]) annotation(
    Line(points = {{-349, -28}, {-375, -28}, {-375, -55}, {-280, -55}}, color = {0, 0, 127}));
  connect(multiplex41.y, attitudeController.u_s) annotation(
    Line(points = {{-256, -62}, {-222, -62}, {-222, -48}}, color = {0, 0, 127}, thickness = 0.5));
  connect(attitudeController.y, deMultiplex3.u) annotation(
    Line(points = {{-188, -48}, {-122, -48}, {-122, -18}}, color = {0, 0, 127}, thickness = 0.5));
  connect(deMultiplex3.y1[1], invAllocMat.tau_x) annotation(
    Line(points = {{-98, -10}, {-74, -10}, {-74, 2}, {-24, 2}}, color = {0, 0, 127}));
  connect(deMultiplex3.y2[1], invAllocMat.tau_y) annotation(
    Line(points = {{-98, -18}, {-44, -18}, {-44, -2}, {-24, -2}}, color = {0, 0, 127}));
  connect(deMultiplex3.y3[1], invAllocMat.tau_z) annotation(
  connect(attitudeCompute.rpy_d, attitudeController.u_s) annotation(
    Line(points = {{-87, -66}, {-45.375, -66}, {-45.375, -62}, {-17.75, -62}}, color = {0, 0, 127}, thickness = 0.5));
  connect(test_yaw.y, attitudeCompute.yaw_d) annotation(
    Line(points = {{-192, -92}, {-129, -92}, {-129, -77}}, color = {0, 0, 127}));
  connect(vel_controller3D.y, attitudeCompute.a_xy) annotation(
    Line(points = {{-206, 28}, {-166, 28}, {-166, -56}, {-128, -56}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    uses(Modelica(version = "4.1.0")),
  Icon(graphics = {Bitmap(extent = {{-140, -140}, {140, 140}}, fileName = "modelica://ModeloDrone/media/drone_white.png")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
  Diagram(coordinateSystem(extent = {{-400, -200}, {400, 200}})));
end Drone;