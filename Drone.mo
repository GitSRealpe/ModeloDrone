within ModeloDrone;

model Drone

  import SI = Modelica.Units.SI;
  parameter SI.Length l = 0.2 "Propeller distance to CoG";
  parameter Real kf = 0.61 / 910^2 "Thrust coefficient [N*s^2/rad^2]";
  parameter Real km = kf * 0.015 "Torque coefficient [N*m*s^2/rad^2]";
  parameter Real w_max(unit = "rad/s") = 1000 "Max rotor speed [rad/s]";
  parameter SI.Mass mass = 0.2 "Drone mass";
  parameter Real payload_mass = 0 "Additional payload mass [kg]";
  parameter Real Ixx = 0.0082, Iyy = 0.0082, Izz = 0.0149 "Drone inertia values";
  parameter SI.Length z_ref = 2 "Altitude setpoint [m]";
  parameter SI.Length x_ref = 0 "X position setpoint [m]";
  parameter SI.Length y_ref = 0 "Y position setpoint [m]";
  parameter Real roll_ref = 0 "Roll angle setpoint [rad]";
  parameter Real pitch_ref = 0 "Pitch angle setpoint [rad]";
  parameter Real yaw_ref = 0 "Yaw angle setpoint [rad]";

  // Sistema eléctrico
  SubSistemas.ModeloBattery battery annotation(
    Placement(transformation(origin = {-140, -120}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.ModeloESC esc1(w_max=w_max) annotation(
    Placement(transformation(origin = {-140, -80}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.ModeloESC esc2(w_max=w_max) annotation(
    Placement(transformation(origin = {-140, -40}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.ModeloESC esc3(w_max=w_max) annotation(
    Placement(transformation(origin = {-140, 0}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.ModeloESC esc4(w_max=w_max) annotation(
    Placement(transformation(origin = {-126, 62}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.ModeloMotor motor1(w_max=w_max) annotation(
    Placement(transformation(origin = {-128, 96}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.ModeloMotor motor2(w_max=w_max) annotation(
    Placement(transformation(origin = {-206, 144}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.ModeloMotor motor3(w_max=w_max) annotation(
    Placement(transformation(origin = {-100, -120}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.ModeloMotor motor4(w_max=w_max) annotation(
    Placement(transformation(origin = {-100, -80}, extent = {{-10, -10}, {10, 10}})));

  // Bloques para sumar corrientes de los 4 motores
  Modelica.Blocks.Math.Add addCurrent1 annotation(
    Placement(transformation(origin = {-60, -120}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add addCurrent2 annotation(
    Placement(transformation(origin = {-60, -80}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add addCurrent3 annotation(
    Placement(transformation(origin = {-60, -40}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.ThrustTiltCompute thrustTilt(mass = mass + payload_mass) annotation(
    Placement(transformation(origin = {-20, -120}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const_ax(k = 0) annotation(
    Placement(transformation(origin = {-40, -120}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const_ay(k = 0) annotation(
    Placement(transformation(origin = {-40, -100}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const_az(k = 0) annotation(
    Placement(transformation(origin = {-40, -80}, extent = {{-10, -10}, {10, 10}})));

  SubSistemas.WindDisturbance wind annotation(
    Placement(transformation(origin = {-140, -160}, extent = {{-10, -10}, {10, 10}})));

  SubSistemas.AllocationMatrix allocMat(l=l, kf=kf, km=km) annotation(
    Placement(transformation(origin = {38, 0}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.invAllocationMatrix invAllocMat(l=l, kf=kf, km=km, w_max=w_max) annotation(
    Placement(transformation(origin = {-12, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Body body(m = mass + payload_mass, I_11 = Ixx, I_22 = Iyy, I_33 = Izz, angles_start (each displayUnit = "rad"), r_CM = {0, 0, 0})  annotation(
    Placement(transformation(origin = {174, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Forces.ForceAndTorque forceAndTorque(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_b)  annotation(
    Placement(transformation(origin = {126, -30}, extent = {{-10, -10}, {10, 10}})));
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
    Placement(transformation(origin = {44, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Routing.Multiplex3 multiplex3 annotation(
    Placement(transformation(origin = {118, 38}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const13(k = 0) annotation(
    Placement(transformation(origin = {38, 32}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const14(k = 0) annotation(
    Placement(transformation(origin = {38, 62}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add addForce annotation(
    Placement(transformation(origin = {82, 6}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add addTorqueX annotation(
    Placement(transformation(origin = {78, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add addTorqueY annotation(
    Placement(transformation(origin = {78, -50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add addTorqueZ annotation(
    Placement(transformation(origin = {78, -70}, extent = {{-10, -10}, {10, 10}})));
  SubSistemas.Controller3D vel_controller3D(controllerType = Modelica.Blocks.Types.SimpleController.PI) annotation(
    Placement(transformation(origin = {-179, 29}, extent = {{-15, -15}, {15, 15}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world)  annotation(
    Placement(transformation(origin = {-10, -60}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  SubSistemas.ForceCompute forceCompute(mass=mass + payload_mass) annotation(
    Placement(transformation(origin = {-120, 28}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant setpoint_z(k = 2) annotation(
    Placement(transformation(origin = {-376, 82}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant setpoint_x(k = 0) annotation(
    Placement(transformation(origin = {-322, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Sources.Constant setpoint_y(k = 0) annotation(
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
  Modelica.Blocks.Sources.Constant setpoint_roll(k = 0) annotation(
    Placement(transformation(origin = {-330, -92}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant setpoint_pitch(k = 0) annotation(
    Placement(transformation(origin = {-338, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Sources.Constant setpoint_yaw(k = 0.1) annotation(
    Placement(transformation(origin = {-358, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Routing.Multiplex3 multiplex41 annotation(
    Placement(transformation(origin = {-268, -62}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Routing.DeMultiplex3 deMultiplex3 annotation(
    Placement(transformation(origin = {-110, -18}, extent = {{-10, -10}, {10, 10}})));

equation
  // Sistema eléctrico: batería → ESC → Motor → AllocMat
  connect(battery.V_terminal, esc1.V_battery);
  connect(battery.V_terminal, esc2.V_battery);
  connect(battery.V_terminal, esc3.V_battery);
  connect(battery.V_terminal, esc4.V_battery);

  connect(invAllocMat.w[1], esc1.w_cmd);
  connect(esc1.V_out, motor1.V_in);
  connect(motor1.w, allocMat.w[1]);

  connect(invAllocMat.w[2], esc2.w_cmd);
  connect(esc2.V_out, motor2.V_in);
  connect(motor2.w, allocMat.w[2]);

  connect(invAllocMat.w[3], esc3.w_cmd);
  connect(esc3.V_out, motor3.V_in);
  connect(motor3.w, allocMat.w[3]);

  connect(invAllocMat.w[4], esc4.w_cmd);
  connect(esc4.V_out, motor4.V_in);
  connect(motor4.w, allocMat.w[4]);

  // Suma de corrientes: motor1.I + motor2.I → addCurrent1
  connect(motor1.I, addCurrent1.u1);
  connect(motor2.I, addCurrent1.u2);
  // addCurrent1.y + motor3.I → addCurrent2
  connect(addCurrent1.y, addCurrent2.u1);
  connect(motor3.I, addCurrent2.u2);
  // addCurrent2.y + motor4.I → addCurrent3
  connect(addCurrent2.y, addCurrent3.u1);
  connect(motor4.I, addCurrent3.u2);
  // addCurrent3.y → batería.I_total
  connect(addCurrent3.y, battery.I_total);

  // ThrustTiltCompute (disponible para estimación de actitud desde aceleración)
  connect(const_ax.y, thrustTilt.a[1]);
  connect(const_ay.y, thrustTilt.a[2]);
  connect(const_az.y, thrustTilt.a[3]);

  // Matriz de asignación y fuerzas/torques
  connect(allocMat.tau[1], addTorqueX.u1);
  connect(wind.tx, addTorqueX.u2);
  connect(allocMat.tau[2], addTorqueY.u1);
  connect(wind.ty, addTorqueY.u2);
  connect(allocMat.tau[3], addTorqueZ.u1);
  connect(wind.tz, addTorqueZ.u2);
  connect(addTorqueX.y, forceAndTorque.torque[1]);
  connect(addTorqueY.y, forceAndTorque.torque[2]);
  connect(addTorqueZ.y, forceAndTorque.torque[3]);
  connect(allocMat.F_z, addForce.u1);
  connect(wind.fz, addForce.u2);
  connect(addForce.y, multiplex3.u3[1]);
  connect(world.frame_b, forceAndTorque.frame_a) annotation(
    Line(points = {{54, -30}, {116, -30}}, color = {95, 95, 95}));
  connect(forceAndTorque.frame_b, body.frame_a) annotation(
    Line(points = {{136, -30}, {164, -30}}, color = {95, 95, 95}));
  connect(const13.y, multiplex3.u2[1]) annotation(
    Line(points = {{49, 32}, {78.5, 32}, {78.5, 38}, {106, 38}}, color = {0, 0, 127}));
  connect(const14.y, multiplex3.u1[1]) annotation(
    Line(points = {{50, 62}, {68, 62}, {68, 45}, {106, 45}}, color = {0, 0, 127}));
  connect(multiplex3.y, forceAndTorque.force) annotation(
    Line(points = {{129, 38}, {119, 38}, {119, -18}, {118, -18}}, color = {0, 0, 127}, thickness = 0.5));
  connect(absoluteVelocity.frame_a, body.frame_a) annotation(
    Line(points = {{0, -60}, {164, -60}, {164, -30}}, color = {95, 95, 95}));
  connect(forceCompute.F, invAllocMat.F_z) annotation(
    Line(points = {{-108.5, 28}, {-40, 28}, {-40, 8}, {-24, 8}}, color = {0, 0, 127}));
  connect(setpoint_z.y, multiplex4.u3[1]) annotation(
    Line(points = {{-376, 71}, {-376, 53}, {-365, 53}}, color = {0, 0, 127}));
  connect(setpoint_y.y, multiplex4.u2[1]) annotation(
    Line(points = {{-345, 84}, {-345, 86}, {-359, 86}, {-359, 54}}, color = {0, 0, 127}));
  connect(setpoint_x.y, multiplex4.u1[1]) annotation(
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
  connect(setpoint_roll.y, multiplex41.u3[1]) annotation(
    Line(points = {{-319, -92}, {-319, -69}, {-280, -69}}, color = {0, 0, 127}));
  connect(setpoint_pitch.y, multiplex41.u2[1]) annotation(
    Line(points = {{-358, -75}, {-358, -64.5}, {-280, -64.5}, {-280, -62}}, color = {0, 0, 127}));
  connect(setpoint_yaw.y, multiplex41.u1[1]) annotation(
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
    Line(points = {{-98, -24}, {-24, -24}, {-24, -8}}, color = {0, 0, 127}));

  annotation(
    uses(Modelica(version = "4.1.0")),
  Icon(graphics = {Bitmap(extent = {{-140, -140}, {140, 140}}, fileName = "modelica://ModeloDrone/media/drone_white.png")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
  Diagram(coordinateSystem(extent = {{-400, -200}, {400, 200}})));
end Drone;
