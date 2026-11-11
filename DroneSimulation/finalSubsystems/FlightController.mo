within DroneSimulation.finalSubsystems;

model FlightController
  import SI = Modelica.Units.SI;
  parameter SI.Mass mass annotation(Dialog(group = "Model"));
  parameter SI.Length l annotation(Dialog(group = "Model"));
  parameter Real kf(unit = "N.s2/rad2") annotation(Dialog(group = "Model"));
  parameter Real km(unit = "N.m.s2/rad2") annotation(Dialog(group = "Model"));
  parameter Real psi(unit = "V.s/rad") annotation(Dialog(group = "Model"));
  parameter SI.Resistance Ra annotation(Dialog(group = "Model"));
  parameter Real maxTilt_deg = 25 annotation(Dialog(group = "Tuning"));
  parameter Real k_pos[3] = {1, 1, 1} annotation(Dialog(group = "Tuning"));
  parameter Real k_vel[3] = {1, 1, 3} annotation(Dialog(group = "Tuning"));
  parameter Real Ti_vel[3] = {1, 1, 2} annotation(Dialog(group = "Tuning"));
  parameter Real k_att[3] = {0.05, 0.05, 0.05} annotation(Dialog(group = "Tuning"));
  parameter Real Td_att[3] = {0.25, 0.25, 0.3} annotation(Dialog(group = "Tuning"));

  Modelica.Blocks.Interfaces.RealInput r_ref[3](each unit = "m") annotation(
    Placement(transformation(extent = {{-140, 40}, {-100, 80}})));
  Modelica.Blocks.Interfaces.RealInput r[3](each unit = "m") annotation(
    Placement(transformation(origin = {-60, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealInput v[3](each unit = "m/s") annotation(
    Placement(transformation(origin = {-20, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealInput angles[3](each unit = "rad") annotation(
    Placement(transformation(origin = {20, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealInput v_bat(unit = "V") annotation(
    Placement(transformation(origin = {60, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealOutput duty[4] annotation(
    Placement(transformation(extent = {{100, -10}, {120, 10}})));

  Controller3D pos_controller3D(controllerType = Modelica.Blocks.Types.SimpleController.P, k = k_pos) annotation(
    Placement(transformation(origin = {-83, 76}, extent = {{-10, -10}, {10, 10}})));
  Controller3D vel_controller3D(controllerType = Modelica.Blocks.Types.SimpleController.PI,
    k = k_vel, Ti = Ti_vel, yMax = {10, 10, 30}) annotation(
    Placement(transformation(origin = {-41, 76}, extent = {{-10, -10}, {10, 10}})));
  ForceCompute forceCompute(mass = mass) annotation(
    Placement(transformation(origin = {18, 60}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Blocks.Math.Gain pitchCmd(k = 1/Modelica.Constants.g_n) annotation(
    Placement(transformation(origin = {-60, 20}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Blocks.Math.Gain rollCmd(k = -1/Modelica.Constants.g_n) annotation(
    Placement(transformation(origin = {-60, 0}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Blocks.Nonlinear.Limiter pitchLimit(uMax = maxTilt_deg*Modelica.Constants.pi/180) annotation(
    Placement(transformation(origin = {-40, 20}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Blocks.Nonlinear.Limiter rollLimit(uMax = maxTilt_deg*Modelica.Constants.pi/180) annotation(
    Placement(transformation(origin = {-40, 0}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Blocks.Sources.Constant yawRef(k = 0) annotation(
    Placement(transformation(origin = {-40, -20}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Blocks.Routing.Multiplex3 attRef annotation(
    Placement(transformation(origin = {-20, 0}, extent = {{-6, -6}, {6, 6}})));
  Controller3D att_controller3D(controllerType = Modelica.Blocks.Types.SimpleController.PD,
    k = k_att, Td = Td_att, yMax = {0.05, 0.05, 0.01}) annotation(
    Placement(transformation(origin = {5, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Routing.DeMultiplex3 demux annotation(
    Placement(transformation(origin = {30, 0}, extent = {{-6, -6}, {6, 6}})));
  MotorMixerVoltage mixer(d = l, kf = kf, km = km, psi = psi, Ra = Ra) annotation(
    Placement(transformation(origin = {74, 0}, extent = {{-19, -19}, {19, 19}})));
equation
  connect(r_ref, pos_controller3D.u_s) annotation(
    Line(points = {{-120, 60}, {-103.5, 60}, {-103.5, 76}, {-95, 76}}, color = {0, 0, 127}, thickness = 0.5));
  connect(r, pos_controller3D.u_m) annotation(
    Line(points = {{-60, -120}, {-60, -80}, {-90, -80}, {-90, 40}, {-83, 40}, {-83, 64}}, color = {0, 0, 127}, thickness = 0.5));
  connect(pos_controller3D.y, vel_controller3D.u_s) annotation(
    Line(points = {{-72, 76}, {-53, 76}}, color = {0, 0, 127}, thickness = 0.5));
  connect(v, vel_controller3D.u_m) annotation(
    Line(points = {{-20, -120}, {-20, -70}, {-85, -70}, {-85, 42}, {-41, 42}, {-41, 64}}, color = {0, 0, 127}, thickness = 0.5));
  connect(vel_controller3D.y[3], forceCompute.a) annotation(
    Line(points = {{-30, 76}, {-12.5, 76}, {-12.5, 60}, {9, 60}}, color = {0, 0, 127}));
  connect(forceCompute.F, mixer.F_z) annotation(
    Line(points = {{27, 60}, {40, 60}, {40, 13}, {52, 13}}, color = {0, 0, 127}));
  connect(vel_controller3D.y[1], pitchCmd.u) annotation(
    Line(points = {{-30, 76}, {-30, 35}, {-75, 35}, {-75, 20}, {-67, 20}}, color = {0, 0, 127}));
  connect(vel_controller3D.y[2], rollCmd.u) annotation(
    Line(points = {{-30, 76}, {-30, 35}, {-75, 35}, {-75, 0}, {-67, 0}}, color = {0, 0, 127}));
  connect(pitchCmd.y, pitchLimit.u) annotation(
    Line(points = {{-53, 20}, {-47, 20}}, color = {0, 0, 127}));
  connect(rollCmd.y, rollLimit.u) annotation(
    Line(points = {{-53, 0}, {-47, 0}}, color = {0, 0, 127}));
  connect(rollLimit.y, attRef.u1[1]) annotation(
    Line(points = {{-33, 0}, {-30, 0}, {-30, 4}, {-27, 4}}, color = {0, 0, 127}));
  connect(pitchLimit.y, attRef.u2[1]) annotation(
    Line(points = {{-33, 20}, {-30, 20}, {-30, 0}, {-27, 0}}, color = {0, 0, 127}));
  connect(yawRef.y, attRef.u3[1]) annotation(
    Line(points = {{-33, -20}, {-30, -20}, {-30, -4}, {-27, -4}}, color = {0, 0, 127}));
  connect(attRef.y, att_controller3D.u_s) annotation(
    Line(points = {{-13, 0}, {-7, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(angles, att_controller3D.u_m) annotation(
    Line(points = {{20, -120}, {20, -40}, {5, -40}, {5, -12}}, color = {0, 0, 127}, thickness = 0.5));
  connect(att_controller3D.y, demux.u) annotation(
    Line(points = {{16, 0}, {23, 0}}, color = {0, 0, 127}, thickness = 0.5));
  connect(demux.y1[1], mixer.tau_x) annotation(
    Line(points = {{37, 4}, {52, 4}}, color = {0, 0, 127}));
  connect(demux.y2[1], mixer.tau_y) annotation(
    Line(points = {{37, 0}, {45, 0}, {45, -6}, {52, -6}}, color = {0, 0, 127}));
  connect(demux.y3[1], mixer.tau_z) annotation(
    Line(points = {{37, -4}, {42, -4}, {42, -15}, {52, -15}}, color = {0, 0, 127}));
  connect(v_bat, mixer.v_bat) annotation(
    Line(points = {{60, -120}, {60, -40}, {74, -40}, {74, -22}}, color = {0, 0, 127}));
  connect(mixer.duty, duty) annotation(
    Line(points = {{95, 0}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
  annotation(
    Icon(graphics = {
      Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 200}, fillColor = {240, 244, 255}, fillPattern = FillPattern.Solid, radius = 10),
      Bitmap(extent = {{-90, 90}, {90, -90}}, fileName = "modelica://DroneSimulation/finalSubsystems/Resources/icon_controller.png"),
      Text(extent = {{-150, 140}, {150, 105}}, textString = "%name", textColor = {0, 0, 200}),
      Text(extent = {{-98, 80}, {-60, 60}}, textString = "ref", textColor = {0, 0, 127}),
      Text(extent = {{-80, -70}, {-40, -95}}, textString = "r", textColor = {0, 0, 127}),
      Text(extent = {{-40, -70}, {0, -95}}, textString = "v", textColor = {0, 0, 127}),
      Text(extent = {{0, -70}, {40, -95}}, textString = "φθψ", textColor = {0, 0, 127}),
      Text(extent = {{40, -70}, {80, -95}}, textString = "V", textColor = {0, 0, 127})}));
end FlightController;