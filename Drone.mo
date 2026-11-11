within ModeloDrone;

model Drone
  import SI = Modelica.Units.SI;
  parameter SI.Length l = 0.2 "Propeller distance to CoG";
  parameter Real kf = 1e-5 "Thrust coefficient";
  parameter Real km = 1e-7 "Torque coefficient";
  parameter Real w_max(unit = "rad/s") = 1000 "Max rotor speed";
  parameter SI.Mass mass = 0.2 "Drone mass";
  parameter Real Ixx = 0.0082, Iyy = 0.0082, Izz = 0.0149 "Drone inertia values";
  SubSistemas.AllocationMatrix allocMat(l = l, kf = kf, km = km) annotation(
    Placement(transformation(origin = {164, 0}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.invAllocationMatrix invAllocMat(l = l, kf = kf, km = km, w_max = w_max) annotation(
    Placement(transformation(origin = {114, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body body(m = mass, I_11 = Ixx, I_22 = Iyy, I_33 = Izz, angles_start(each displayUnit = "rad"), r_CM = {0, 0, 0}) annotation(
    Placement(transformation(origin = {300, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Forces.ForceAndTorque forceAndTorque(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_b) annotation(
    Placement(transformation(origin = {252, -30}, extent = {{-10, -10}, {10, 10}})));
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}, animateGravity = true) annotation(
    Placement(transformation(origin = {170, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Routing.Multiplex3 multiplex3 annotation(
    Placement(transformation(origin = {222, 14}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const13(k = 0) annotation(
    Placement(transformation(origin = {164, 32}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const14(k = 0) annotation(
    Placement(transformation(origin = {164, 62}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.Controller3D vel_controller3D(controllerType = Modelica.Blocks.Types.SimpleController.PI) annotation(
    Placement(transformation(origin = {-223, 27}, extent = {{-15, 15}, {15, -15}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {122, 96}, extent = {{10, -10}, {-10, 10}})));
  SubSistemas.ForceCompute forceCompute(mass = mass) annotation(
    Placement(transformation(origin = {-57, 21}, extent = {{-17, -17}, {17, 17}})));
  Modelica.Blocks.Sources.Constant testz(k = 1) annotation(
    Placement(transformation(origin = {-382, 80}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant testx(k = 1) annotation(
    Placement(transformation(origin = {-284, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Sources.Constant testy(k = 0) annotation(
    Placement(transformation(origin = {-326, 126}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Routing.Multiplex3 multiplex4 annotation(
    Placement(transformation(origin = {-358, 42}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {-54, -154}, extent = {{10, -10}, {-10, 10}})));
  ModeloDrone.SubSistemas.Controller3D pos_controller3D(controllerType = Modelica.Blocks.Types.SimpleController.PD, Td = {0.7, 0.1, 0.5}, k = {0.7, 1, 1}) annotation(
    Placement(transformation(origin = {-277, 27}, extent = {{-15, -15}, {15, 15}})));
  SubSistemas.AttitudeController attitudeController(Ixx = Ixx, Iyy = Iyy, Izz = Izz) annotation(
    Placement(transformation(origin = {2, -62}, extent = {{-16, -16}, {16, 16}})));
  SubSistemas.OrientationSensor orientationSensor annotation(
    Placement(transformation(origin = {107, -109}, extent = {{13, -13}, {-13, 13}})));
  Modelica.Blocks.Routing.DeMultiplex3 deMultiplex3 annotation(
    Placement(transformation(origin = {70, -20}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.AttitudeCompute attitudeCompute annotation(
    Placement(transformation(origin = {-108, -66}, extent = {{-18, -18}, {18, 18}})));
  Modelica.Blocks.Sources.Constant test_yaw(k = 0)  annotation(
    Placement(transformation(origin = {-204, -92}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(invAllocMat.w, allocMat.w) annotation(
    Line(points = {{125, 0}, {151, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(allocMat.tau, forceAndTorque.torque) annotation(
    Line(points = {{175.5, -6}, {251.5, -6}, {251.5, -18}}, color = {0, 0, 127}, thickness = 0.5));
  connect(world.frame_b, forceAndTorque.frame_a) annotation(
    Line(points = {{180, -30}, {242, -30}}, color = {95, 95, 95}));
  connect(forceAndTorque.frame_b, body.frame_a) annotation(
    Line(points = {{262, -30}, {290, -30}}, color = {95, 95, 95}));
  connect(allocMat.F_z, multiplex3.u3[1]) annotation(
    Line(points = {{175.5, 6}, {190.5, 6}, {190.5, 7}, {209.5, 7}}, color = {0, 0, 127}));
  connect(const13.y, multiplex3.u2[1]) annotation(
    Line(points = {{175, 32}, {186, 32}, {186, 14}, {210, 14}}, color = {0, 0, 127}));
  connect(const14.y, multiplex3.u1[1]) annotation(
    Line(points = {{175, 62}, {193, 62}, {193, 22}, {209, 22}}, color = {0, 0, 127}));
  connect(multiplex3.y, forceAndTorque.force) annotation(
    Line(points = {{233, 14}, {243, 14}, {243, -18}}, color = {0, 0, 127}, thickness = 0.5));
  connect(absoluteVelocity.frame_a, body.frame_a) annotation(
    Line(points = {{132, 96}, {290, 96}, {290, -30}}, color = {95, 95, 95}));
  connect(testz.y, multiplex4.u3[1]) annotation(
    Line(points = {{-382, 69}, {-382, 53}, {-365, 53}}, color = {0, 0, 127}));
  connect(testy.y, multiplex4.u2[1]) annotation(
    Line(points = {{-337, 126}, {-359, 126}, {-359, 54}}, color = {0, 0, 127}));
  connect(testx.y, multiplex4.u1[1]) annotation(
    Line(points = {{-295, 82}, {-340.5, 82}, {-340.5, 54}, {-352, 54}}, color = {0, 0, 127}));
  connect(absoluteVelocity.v, vel_controller3D.u_m) annotation(
    Line(points = {{111, 96}, {-167, 96}, {-167, 45}, {-223, 45}}, color = {0, 0, 127}, thickness = 0.5));
  connect(absolutePosition.frame_a, body.frame_a) annotation(
    Line(points = {{-44, -154}, {290, -154}, {290, -30}}, color = {95, 95, 95}));
  connect(pos_controller3D.y, vel_controller3D.u_s) annotation(
    Line(points = {{-260.5, 27}, {-241, 27}}, color = {0, 0, 127}, thickness = 0.5));
  connect(multiplex4.y, pos_controller3D.u_s) annotation(
    Line(points = {{-358, 32}, {-358, 27}, {-295, 27}}, color = {0, 0, 127}, thickness = 0.5));
  connect(orientationSensor.frame_a, body.frame_a) annotation(
    Line(points = {{123, -109}, {123, -30}, {289.6, -30}}, color = {95, 95, 95}));
  connect(attitudeController.y, deMultiplex3.u) annotation(
    Line(points = {{20, -62}, {20, -20}, {58, -20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(deMultiplex3.y1[1], invAllocMat.tau_x) annotation(
    Line(points = {{81, -13}, {52, -13}, {52, 2}, {102, 2}}, color = {0, 0, 127}));
  connect(deMultiplex3.y2[1], invAllocMat.tau_y) annotation(
    Line(points = {{81, -20}, {82, -20}, {82, -2}, {102, -2}}, color = {0, 0, 127}));
  connect(deMultiplex3.y3[1], invAllocMat.tau_z) annotation(
    Line(points = {{81, -27}, {81, -8}, {102, -8}}, color = {0, 0, 127}));
  connect(absolutePosition.r, pos_controller3D.u_m) annotation(
    Line(points = {{-65, -154}, {-277, -154}, {-277, 9}}, color = {0, 0, 127}, thickness = 0.5));
  connect(attitudeCompute.rpy_d, attitudeController.u_s) annotation(
    Line(points = {{-87, -66}, {-45.375, -66}, {-45.375, -62}, {-17.75, -62}}, color = {0, 0, 127}, thickness = 0.5));
  connect(orientationSensor.R, attitudeController.R_m) annotation(
    Line(points = {{92, -102}, {-8, -102}, {-8, -82}}, color = {0, 0, 127}, thickness = 0.5));
  connect(orientationSensor.Omega, attitudeController.w_m) annotation(
    Line(points = {{92, -116}, {12, -116}, {12, -82}}, color = {0, 0, 127}, thickness = 0.5));
  connect(orientationSensor.R, forceCompute.R_m) annotation(
    Line(points = {{92, -102}, {-57, -102}, {-57, 1}}, color = {0, 0, 127}, thickness = 0.5));
  connect(forceCompute.T, invAllocMat.F_z) annotation(
    Line(points = {{-38, 20}, {102, 20}, {102, 8}}, color = {0, 0, 127}));
  connect(vel_controller3D.y, forceCompute.a_cmd) annotation(
    Line(points = {{-206, 28}, {-76, 28}, {-76, 22}}, color = {0, 0, 127}, thickness = 0.5));
  connect(test_yaw.y, attitudeCompute.yaw_d) annotation(
    Line(points = {{-192, -92}, {-129, -92}, {-129, -77}}, color = {0, 0, 127}));
  connect(vel_controller3D.y, attitudeCompute.a_xy) annotation(
    Line(points = {{-206, 28}, {-166, 28}, {-166, -56}, {-128, -56}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    uses(Modelica(version = "4.1.0")),
    Icon(graphics = {Bitmap(extent = {{-140, -140}, {140, 140}}, fileName = "modelica://ModeloDrone/media/drone_white.png")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
    Diagram(coordinateSystem(extent = {{-400, -200}, {400, 200}})));
end Drone;