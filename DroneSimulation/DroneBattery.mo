within DroneSimulation;

model DroneBattery
  import SI = Modelica.Units.SI;
  parameter SI.Position x_ref = 2 annotation(Dialog(group = "Experiment"));
  parameter SI.Position y_ref = 3 annotation(Dialog(group = "Experiment"));
  parameter SI.Height z_ref = 10 annotation(Dialog(group = "Experiment"));
  parameter Real Q_bat_Ah = 1.0 annotation(Dialog(group = "Experiment"));
  parameter Integer N_cells = 2 annotation(Dialog(group = "Experiment"));
  parameter SI.Mass m_payload(displayUnit = "g") = 0 annotation(Dialog(group = "Experiment"));
  parameter SI.Velocity V_wind = 3 annotation(Dialog(group = "Wind"));
  parameter Real wind_dir_deg = 0 annotation(Dialog(group = "Wind"));
  parameter SI.Velocity V_gust = 3 annotation(Dialog(group = "Wind"));
  parameter SI.Time t_gust = 60 annotation(Dialog(group = "Wind"));
  parameter SI.Velocity sigma_turb = 0.5 annotation(Dialog(group = "Wind"));
  parameter Integer seed = 1 annotation(Dialog(group = "Wind"));
  parameter SI.Area CdA_h = 0.015 annotation(Dialog(group = "Wind"));
  parameter Real maxTilt_deg = 25 annotation(Dialog(group = "Controller"));

  Modelica.Blocks.Sources.Constant testx(k = x_ref) annotation(
    Placement(transformation(origin = {-185, 55}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Blocks.Sources.Constant testy(k = y_ref) annotation(
    Placement(transformation(origin = {-185, 35}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Blocks.Sources.Constant testz(k = z_ref) annotation(
    Placement(transformation(origin = {-185, 15}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Blocks.Routing.Multiplex3 multiplex3 annotation(
    Placement(transformation(origin = {-150, 35}, extent = {{-8, -8}, {8, 8}})));

  finalSubsystems.FlightController controller(mass = plant.mass, l = plant.l, kf = plant.kf, km = plant.km, psi = plant.psi, Ra = plant.Ra,
    maxTilt_deg = maxTilt_deg) annotation(
    Placement(transformation(origin = {-80, 35}, extent = {{-25, -25}, {25, 25}})));

  finalSubsystems.DronePlant plant(Q_bat_Ah = Q_bat_Ah, N_cells = N_cells) annotation(
    Placement(transformation(origin = {40, 35}, extent = {{-25, -25}, {25, 25}})));
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {180, -95}, extent = {{-8, -8}, {8, 8}})));

  finalSubsystems.Disturbances disturbances(m_payload = m_payload, V_wind = V_wind, wind_dir_deg = wind_dir_deg, V_gust = V_gust, t_gust = t_gust,
    sigma_turb = sigma_turb, seed = seed, CdA_h = CdA_h, rho_air = plant.rho_air) annotation(
    Placement(transformation(origin = {150, 35}, extent = {{20, -20}, {-20, 20}})));

  finalSubsystems.Sensors feedback annotation(
    Placement(transformation(origin = {40, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 180)));

  Modelica.Blocks.Logical.TerminateSimulation endOfFlight(
    condition = (time > 20 and feedback.r[3] < 0.75*z_ref) or plant.battery.SOC < 0.02 or (time > 5 and feedback.v_bat < plant.N_cells*plant.V_cell_cutoff),
    terminationText = "End of flight") annotation(
    Placement(transformation(origin = {-152, -91}, extent = {{-40, -6}, {40, 6}})));
equation
  connect(testx.y, multiplex3.u1[1]) annotation(
    Line(points = {{-176, 55}, {-168, 55}, {-168, 41}, {-160, 41}}, color = {0, 0, 127}));
  connect(testy.y, multiplex3.u2[1]) annotation(
    Line(points = {{-176, 35}, {-160, 35}}, color = {0, 0, 127}));
  connect(testz.y, multiplex3.u3[1]) annotation(
    Line(points = {{-176, 15}, {-168, 15}, {-168, 29}, {-160, 29}}, color = {0, 0, 127}));
  connect(multiplex3.y, controller.r_ref) annotation(
    Line(points = {{-141, 35}, {-125, 35}, {-125, 50}, {-110, 50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(controller.duty, plant.duty) annotation(
    Line(points = {{-52.5, 35}, {10, 35}}, color = {0, 0, 127}, thickness = 0.5));
  connect(disturbances.frame_a, plant.airframe) annotation(
    Line(points = {{130, 35}, {65, 35}}, color = {95, 95, 95}, thickness = 0.5));
  connect(plant.airframe, feedback.frame_a) annotation(
    Line(points = {{65, 35}, {90, 35}, {90, -60}, {60, -60}}, color = {95, 95, 95}, thickness = 0.5));
  connect(plant.bat_p, feedback.bat_p) annotation(
    Line(points = {{30, 10}, {30, -20}, {48, -20}, {48, -40}}, color = {0, 0, 255}));
  connect(plant.bat_n, feedback.bat_n) annotation(
    Line(points = {{50, 10}, {50, -20}, {32, -20}, {32, -40}}, color = {0, 0, 255}));
  connect(feedback.r, controller.r) annotation(
    Line(points = {{52, -82}, {52, -100}, {-95, -100}, {-95, 5}}, color = {0, 0, 127}, thickness = 0.5));
  connect(feedback.v, controller.v) annotation(
    Line(points = {{44, -82}, {44, -96}, {-85, -96}, {-85, 5}}, color = {0, 0, 127}, thickness = 0.5));
  connect(feedback.angles, controller.angles) annotation(
    Line(points = {{36, -82}, {36, -92}, {-75, -92}, {-75, 5}}, color = {0, 0, 127}, thickness = 0.5));
  connect(feedback.v_bat, controller.v_bat) annotation(
    Line(points = {{28, -82}, {28, -88}, {-65, -88}, {-65, 5}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "4.1.0"), DroneLibrary(version = "1.0.0")),
    experiment(StartTime = 0, StopTime = 7200, Tolerance = 1e-06, Interval = 0.5),
    Icon(graphics = {Bitmap(extent = {{-140, -140}, {140, 140}}, fileName = "modelica://DroneSimulation/finalSubsystems/Resources/drone_colors.png")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
    Diagram(coordinateSystem(extent = {{-200, -120}, {200, 100}}), graphics = {
      Rectangle(lineColor = {0, 140, 0}, fillColor = {235, 250, 235}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{-198, 80}, {-130, -10}}, radius = 4),
      Text(textColor = {0, 140, 0}, extent = {{-196, 78}, {-132, 68}}, textString = "REFERENCE"),
      Rectangle(lineColor = {0, 0, 200}, fillColor = {235, 240, 255}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{-125, 80}, {-35, -10}}, radius = 4),
      Text(textColor = {0, 0, 200}, extent = {{-123, 78}, {-37, 68}}, textString = "CONTROLLER"),
      Rectangle(lineColor = {200, 0, 0}, fillColor = {255, 238, 238}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{-5, 80}, {85, -10}}, radius = 4),
      Text(textColor = {200, 0, 0}, extent = {{-3, 78}, {83, 68}}, textString = "PLANT"),
      Rectangle(lineColor = {0, 100, 200}, fillColor = {235, 245, 255}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{110, 80}, {195, -10}}, radius = 4),
      Text(textColor = {0, 100, 200}, extent = {{112, 78}, {193, 68}}, textString = "DISTURBANCES"),
      Rectangle(lineColor = {150, 100, 0}, fillColor = {255, 248, 230}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{-5, -20}, {85, -110}}, radius = 4),
      Text(textColor = {150, 100, 0}, extent = {{-3, -100}, {83, -110}}, textString = "FEEDBACK")}));
end DroneBattery;