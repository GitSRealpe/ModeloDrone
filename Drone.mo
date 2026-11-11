within ModeloDrone;

model Drone "Quadcopter: cascaded position/velocity/attitude control + motors, battery and rigid-body plant"
  import SI = Modelica.Units.SI;
  // ---------------------------------------------------------------- Parameters
  parameter SI.Length l = 0.2 "Propeller distance to CoG";
  parameter Real kf = 1e-5 "Thrust coefficient";
  parameter Real km = 1e-7 "Torque coefficient";
  parameter Real w_max(unit = "rad/s") = 1000 "Max rotor speed";
  parameter SI.Mass mass = 0.2 "Drone mass";
  parameter Real Ixx = 0.0082, Iyy = 0.0082, Izz = 0.0149 "Drone inertia values";
  parameter SI.Mass m_payload = 0 "Carried payload (m_total = mass + m_payload); unknown to the controller" annotation(
    Dialog(group = "Carga"));
  // Motors (equivalent DC model, same for the 4 rotors)
  parameter SI.Resistance R_mot = 0.1 "Motor armature resistance" annotation(
    Dialog(group = "Motores"));
  parameter SI.Inductance L_mot = 1e-4 "Motor armature inductance" annotation(
    Dialog(group = "Motores"));
  parameter Real ke(unit = "V.s/rad") = 0.005 "Back-EMF constant (= kt)" annotation(
    Dialog(group = "Motores"));
  parameter SI.Inertia Jm = 2e-5 "Rotor inertia (motor + propeller)" annotation(
    Dialog(group = "Motores"));
  parameter Real bm(unit = "N.m.s/rad") = 1e-6 "Motor viscous friction" annotation(
    Dialog(group = "Motores"));
  parameter Boolean motorsSteadyStart = true "Start motors and ESC in steady state" annotation(
    Dialog(group = "Motores"));
  parameter SI.AngularVelocity w_hover = sqrt((mass + m_payload)*Modelica.Constants.g_n/(4*kf)) "Hover rotor speed (initial guess / start)" annotation(
    Dialog(group = "Motores"));
  // Battery
  parameter SI.Voltage V_bat = 7.4 "Battery nominal voltage (2S LiPo)" annotation(
    Dialog(group = "Bateria"));
  parameter Real C_bat(unit = "A.h") = 0.5 "Battery capacity" annotation(
    Dialog(group = "Bateria"));
  // Disturbances
  parameter Real c_d(unit = "N.s/m") = 0 "Linear aerodynamic drag coefficient" annotation(
    Dialog(group = "Perturbaciones"));
  parameter SI.Force F_wind[3] = {0, 0, 0} "Wind force, world frame" annotation(
    Dialog(group = "Perturbaciones"));
  parameter SI.Time t_wind = 0 "Wind start time" annotation(
    Dialog(group = "Perturbaciones"));
  // ---------------------------------------------------------------- Setpoints
  Modelica.Blocks.Sources.Constant testx(k = 1) annotation(
    Placement(transformation(origin = {-370, 80}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant testy(k = 0) annotation(
    Placement(transformation(origin = {-370, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant testz(k = 1) annotation(
    Placement(transformation(origin = {-370, 20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Routing.Multiplex3 multiplex4 annotation(
    Placement(transformation(origin = {-320, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant test_yaw(k = 0) annotation(
    Placement(transformation(origin = {-105, -2}, extent = {{-10, -10}, {10, 10}})));
  // ---------------------------------------------------------------- Outer loop (position -> velocity -> acceleration)
  ModeloDrone.SubSistemas.Controller3D pos_controller3D(controllerType = Modelica.Blocks.Types.SimpleController.PD, Td = {0.7, 0.1, 0.5}, k = {0.7, 1, 1}) annotation(
    Placement(transformation(origin = {-260, 50}, extent = {{-20, -20}, {20, 20}})));
  SubSistemas.Controller3D vel_controller3D(controllerType = Modelica.Blocks.Types.SimpleController.PI) annotation(
    Placement(transformation(origin = {-180, 50}, extent = {{-20, -20}, {20, 20}})));
  // ---------------------------------------------------------------- Thrust & attitude control
  SubSistemas.ForceCompute forceCompute(mass = mass) "Uses the nominal mass (payload unknown)" annotation(
    Placement(transformation(origin = {-60, 90}, extent = {{-20, -20}, {20, 20}})));
  SubSistemas.AttitudeCompute attitudeCompute annotation(
    Placement(transformation(origin = {-60, 10}, extent = {{-20, -20}, {20, 20}})));
  SubSistemas.AttitudeController attitudeController(Ixx = Ixx, Iyy = Iyy, Izz = Izz) annotation(
    Placement(transformation(origin = {40, 10}, extent = {{-20, -20}, {20, 20}})));
  // ---------------------------------------------------------------- Control allocation
  Modelica.Blocks.Routing.DeMultiplex3 deMultiplex3 annotation(
    Placement(transformation(origin = {100, 10}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.invAllocationMatrix invAllocMat(l = l, kf = kf, km = km, w_max = w_max) annotation(
    Placement(transformation(origin = {170, 23}, extent = {{-30, -30}, {30, 30}})));
  // ---------------------------------------------------------------- Electrical subsystem: ESC + motors + battery
  SubSistemas.ESC esc(R = R_mot, ke = ke, bm = bm, km = km, steadyStateInit = motorsSteadyStart) annotation(
    Placement(transformation(origin = {244, 37}, extent = {{-20, -20}, {20, 20}})));
  SubSistemas.ModeloMotor motor[4](each R = R_mot, each L = L_mot, each ke = ke, each Jm = Jm, each bm = bm, each km = km, each steadyStateInit = motorsSteadyStart, each w_start = w_hover) annotation(
    Placement(transformation(origin = {350, 11}, extent = {{-20, -20}, {20, 20}})));
  SubSistemas.Bateria bateria(V_nom = V_bat, C_Ah = C_bat) annotation(
    Placement(transformation(origin = {300, -70}, extent = {{-20, -20}, {20, 20}})));
  // ---------------------------------------------------------------- Plant
  SubSistemas.AllocationMatrix allocMat(l = l, kf = kf, km = km) annotation(
    Placement(transformation(origin = {446, 19}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Sources.Constant const14(k = 0) "Body-x force" annotation(
    Placement(transformation(origin = {495, 100}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const13(k = 0) "Body-y force" annotation(
    Placement(transformation(origin = {495, 70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Routing.Multiplex3 multiplex3 annotation(
    Placement(transformation(origin = {540, 70}, extent = {{-10, -10}, {10, 10}})));
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}, animateGravity = true) annotation(
    Placement(transformation(origin = {470, -36}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Forces.ForceAndTorque forceAndTorque(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_b) annotation(
    Placement(transformation(origin = {580, -20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body body(m = mass + m_payload, I_11 = Ixx, I_22 = Iyy, I_33 = Izz, angles_start(each displayUnit = "rad"), r_CM = {0, 0, 0}, r_0(each fixed = true), v_0(each fixed = true), angles_fixed = true, w_0_fixed = true) annotation(
    Placement(transformation(origin = {680, -14}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.Perturbaciones perturbaciones(c_d = c_d, F_wind = F_wind, t_wind = t_wind) annotation(
    Placement(transformation(origin = {644, -90}, extent = {{-20, -20}, {20, 20}})));
  // ---------------------------------------------------------------- Sensors
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {-230, -130}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {-150, -130}, extent = {{10, -10}, {-10, 10}})));
  SubSistemas.OrientationSensor orientationSensor annotation(
    Placement(transformation(origin = {-31, -131}, extent = {{-29, -29}, {29, 29}})));
equation
// Setpoints
  connect(testx.y, multiplex4.u1[1]) annotation(
    Line(points = {{-359, 80}, {-345, 80}, {-345, 57}, {-332, 57}}, color = {0, 0, 127}));
  connect(testy.y, multiplex4.u2[1]) annotation(
    Line(points = {{-359, 50}, {-332, 50}}, color = {0, 0, 127}));
  connect(testz.y, multiplex4.u3[1]) annotation(
    Line(points = {{-359, 20}, {-345, 20}, {-345, 43}, {-332, 43}}, color = {0, 0, 127}));
  connect(test_yaw.y, attitudeCompute.yaw_d) annotation(
    Line(points = {{-94, -2}, {-83, -2}}, color = {0, 0, 127}));
// Outer loop
  connect(multiplex4.y, pos_controller3D.u_s) annotation(
    Line(points = {{-309, 50}, {-284, 50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(pos_controller3D.y, vel_controller3D.u_s) annotation(
    Line(points = {{-238, 50}, {-204, 50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vel_controller3D.y, forceCompute.a_cmd) annotation(
    Line(points = {{-158, 50}, {-130, 50}, {-130, 90}, {-83, 90}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vel_controller3D.y, attitudeCompute.a_xy) annotation(
    Line(points = {{-158, 50}, {-130, 50}, {-130, 22}, {-83, 22}}, color = {0, 0, 127}, thickness = 0.5));
// Attitude control
  connect(attitudeCompute.rpy_d, attitudeController.u_s) annotation(
    Line(points = {{-37, 10}, {16, 10}}, color = {0, 0, 127}, thickness = 0.5));
  connect(attitudeController.y, deMultiplex3.u) annotation(
    Line(points = {{62, 10}, {88, 10}}, color = {0, 0, 127}, thickness = 0.5));
// Control allocation
  connect(forceCompute.T, invAllocMat.F_z) annotation(
    Line(points = {{-37, 89.6}, {128, 89.6}, {128, 44}, {135.5, 44}}, color = {0, 0, 127}));
  connect(deMultiplex3.y1[1], invAllocMat.tau_x) annotation(
    Line(points = {{111, 17}, {124, 17}, {124, 29}, {135.5, 29}}, color = {0, 0, 127}));
  connect(deMultiplex3.y2[1], invAllocMat.tau_y) annotation(
    Line(points = {{111, 10}, {127.25, 10}, {127.25, 14}, {135.5, 14}}, color = {0, 0, 127}));
  connect(deMultiplex3.y3[1], invAllocMat.tau_z) annotation(
    Line(points = {{111, 3}, {128.25, 3}, {128.25, -1}, {135.5, -1}}, color = {0, 0, 127}));
// Electrical subsystem: w* -> ESC -> V -> motors -> w
  connect(invAllocMat.w, esc.w_ref) annotation(
    Line(points = {{203, 23}, {210.5, 23}, {210.5, 37}, {220, 37}}, color = {0, 0, 127}, thickness = 0.5));
  connect(esc.V, motor.V) annotation(
    Line(points = {{266, 37}, {289, 37}, {289, 11}, {326, 11}}, color = {0, 0, 127}, thickness = 0.5));
  connect(motor.w, allocMat.w) annotation(
    Line(points = {{372, 19}, {423, 19}}, color = {0, 0, 127}, thickness = 0.5));
  connect(motor.w, esc.w_m) annotation(
    Line(points = {{372, 19}, {386, 19}, {386, 61}, {258, 61}}, color = {0, 0, 127}, thickness = 0.5));
  connect(esc.V, bateria.V) annotation(
    Line(points = {{266, 37}, {266, -46}, {290, -46}}, color = {0, 0, 127}, thickness = 0.5));
  connect(motor.i, bateria.i) annotation(
    Line(points = {{372, 3}, {380, 3}, {380, -30}, {310, -30}, {310, -46}}, color = {0, 0, 127}, thickness = 0.5));
  connect(bateria.V_bat, esc.V_bat) annotation(
    Line(points = {{278, -70}, {278, 13}, {244, 13}}, color = {0, 0, 127}));
// Plant
  connect(const14.y, multiplex3.u1[1]) annotation(
    Line(points = {{506, 100}, {518, 100}, {518, 77}, {528, 77}}, color = {0, 0, 127}));
  connect(const13.y, multiplex3.u2[1]) annotation(
    Line(points = {{506, 70}, {528, 70}}, color = {0, 0, 127}));
  connect(allocMat.F_z, multiplex3.u3[1]) annotation(
    Line(points = {{469, 31}, {518, 31}, {518, 63}, {528, 63}}, color = {0, 0, 127}));
  connect(multiplex3.y, forceAndTorque.force) annotation(
    Line(points = {{551, 70}, {572, 70}, {572, -8}}, color = {0, 0, 127}, thickness = 0.5));
  connect(allocMat.tau, forceAndTorque.torque) annotation(
    Line(points = {{469, 7}, {469, -8}, {580, -8}}, color = {0, 0, 127}, thickness = 0.5));
  connect(world.frame_b, forceAndTorque.frame_a) annotation(
    Line(points = {{480, -36}, {560, -36}, {560, -20}, {570, -20}}, color = {95, 95, 95}));
  connect(forceAndTorque.frame_b, body.frame_a) annotation(
    Line(points = {{590, -20}, {626, -20}, {626, -14}, {670, -14}}, color = {95, 95, 95}));
  connect(perturbaciones.frame_b, body.frame_a) annotation(
    Line(points = {{664, -90}, {664, -59}, {670, -59}, {670, -14}}, color = {95, 95, 95}));
// Sensors (all share the bus along the bottom of the diagram)
  connect(absolutePosition.frame_a, body.frame_a) annotation(
    Line(points = {{-220, -130}, {-210, -130}, {-210, -170}, {600, -170}, {600, -14}, {670, -14}}, color = {95, 95, 95}));
  connect(absoluteVelocity.frame_a, body.frame_a) annotation(
    Line(points = {{-140, -130}, {-130, -130}, {-130, -170}, {600, -170}, {600, -14}, {670, -14}}, color = {95, 95, 95}));
  connect(orientationSensor.frame_a, body.frame_a) annotation(
    Line(points = {{-66, -131}, {-34, -131}, {-34, -170}, {600, -170}, {600, -14}, {670, -14}}, color = {95, 95, 95}));
// Feedback
  connect(absolutePosition.r, pos_controller3D.u_m) annotation(
    Line(points = {{-241, -130}, {-260, -130}, {-260, 26}}, color = {0, 0, 127}, thickness = 0.5));
  connect(absoluteVelocity.v, vel_controller3D.u_m) annotation(
    Line(points = {{-161, -130}, {-180, -130}, {-180, 26}}, color = {0, 0, 127}, thickness = 0.5));
  connect(orientationSensor.R, attitudeController.R_m) annotation(
    Line(points = {{1, -114}, {26.8, -114}, {26.8, -14}}, color = {0, 0, 127}, thickness = 0.5));
  connect(orientationSensor.Omega, attitudeController.w_m) annotation(
    Line(points = {{1, -148}, {53.2, -148}, {53.2, -14}}, color = {0, 0, 127}, thickness = 0.5));
  connect(orientationSensor.R, forceCompute.R_m) annotation(
    Line(points = {{1, -114}, {26.8, -114}, {26.8, -30}, {-10, -30}, {-10, 50}, {-60, 50}, {-60, 66}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    uses(Modelica(version = "4.1.0")),
    experiment(StopTime = 10, Interval = 0.01),
    Icon(graphics = {Bitmap(extent = {{-140, -140}, {140, 140}}, fileName = "modelica://ModeloDrone/media/drone_white.png")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
    Diagram(coordinateSystem(extent = {{-400, -200}, {640, 200}}), graphics = {Text(textColor = {64, 64, 64}, extent = {{-390, 190}, {630, 160}}, textString = "Quadcopter - cascaded control", textStyle = {TextStyle.Bold}), Rectangle(lineColor = {160, 160, 164}, fillColor = {245, 245, 245}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{-392, 110}, {-300, 5}}, radius = 4), Text(textColor = {100, 100, 100}, extent = {{-388, 108}, {-304, 100}}, textString = "Referencias", horizontalAlignment = TextAlignment.Left), Rectangle(lineColor = {160, 160, 164}, fillColor = {240, 246, 255}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{-294, 98}, {-140, 12}}, radius = 4), Text(textColor = {100, 100, 100}, extent = {{-290, 96}, {-144, 88}}, textString = "Lazo externo: posicion / velocidad", horizontalAlignment = TextAlignment.Left), Rectangle(lineColor = {160, 160, 164}, fillColor = {240, 255, 244}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{-122, 132}, {74, -24}}, radius = 4), Text(textColor = {100, 100, 100}, extent = {{-118, 130}, {70, 122}}, textString = "Empuje y actitud", horizontalAlignment = TextAlignment.Left), Rectangle(lineColor = {160, 160, 164}, fillColor = {255, 250, 238}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{82, 80}, {208, -24}}, radius = 4), Text(textColor = {100, 100, 100}, extent = {{86, 78}, {204, 70}}, textString = "Asignacion de control", horizontalAlignment = TextAlignment.Left), Rectangle( lineColor = {160, 160, 164}, fillColor = {255, 255, 232}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{214, 66}, {404, -100}}, radius = 4), Text(textColor = {100, 100, 100}, extent = {{218, 64}, {400, 56}}, textString = "Sistema electrico: ESC, motores y bateria", horizontalAlignment = TextAlignment.Left), Rectangle(lineColor = {160, 160, 164}, fillColor = {255, 242, 242}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{454, 122}, {636, -40}}, radius = 4), Text(textColor = {100, 100, 100}, extent = {{458, 120}, {632, 112}}, textString = "Planta", horizontalAlignment = TextAlignment.Left), Rectangle(lineColor = {160, 160, 164}, fillColor = {242, 242, 242}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{544, -46}, {636, -110}}, radius = 4), Text(textColor = {100, 100, 100}, extent = {{548, -102}, {632, -110}}, textString = "Perturbaciones", horizontalAlignment = TextAlignment.Left), Rectangle(lineColor = {160, 160, 164}, fillColor = {246, 242, 255}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{-252, -100}, {36, -158}}, radius = 4), Text(textColor = {100, 100, 100}, extent = {{-248, -102}, {32, -110}}, textString = "Sensores", horizontalAlignment = TextAlignment.Left)}));
end Drone;