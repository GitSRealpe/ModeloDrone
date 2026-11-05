within DroneSimulation.finalSubsystems;

model WindProfile
  import SI = Modelica.Units.SI;
  import Modelica.Constants.pi;
  parameter SI.Velocity V_mean = 3 annotation(Dialog(group = "Mean wind"));
  parameter Real direction_deg = 0 annotation(Dialog(group = "Mean wind"));
  parameter SI.Velocity V_gust = 3 annotation(Dialog(group = "Gust"));
  parameter SI.Time t_gust = 30 annotation(Dialog(group = "Gust"));
  parameter SI.Time rise_gust = 2 annotation(Dialog(group = "Gust"));
  parameter SI.Time hold_gust = 5 annotation(Dialog(group = "Gust"));
  parameter SI.Time fall_gust = 3 annotation(Dialog(group = "Gust"));
  parameter SI.Velocity sigma_turb = 0.5 annotation(Dialog(group = "Turbulence"));
  parameter SI.Time T_turb = 5 annotation(Dialog(group = "Turbulence"));
  parameter SI.Time Ts_noise = 0.5 annotation(Dialog(group = "Turbulence"));
  parameter Integer seed = 1 annotation(Dialog(group = "Turbulence"));
  final parameter Real sigma_white = sigma_turb*sqrt(2*T_turb/Ts_noise);

  Modelica.Blocks.Interfaces.RealOutput v_wind[3](each unit = "m/s") annotation(
    Placement(transformation(extent = {{100, -10}, {120, 10}})));

  Modelica.Blocks.Sources.Constant mean(k = V_mean) annotation(
    Placement(transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Trapezoid gust(amplitude = V_gust, rising = rise_gust, width = hold_gust, falling = fall_gust,
    period = 1e9, nperiod = 1, startTime = t_gust) annotation(
    Placement(transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add speed annotation(
    Placement(transformation(origin = {-30, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Gain toX(k = cos(direction_deg*pi/180)) annotation(
    Placement(transformation(origin = {10, 70}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Blocks.Math.Gain toY(k = sin(direction_deg*pi/180)) annotation(
    Placement(transformation(origin = {10, 30}, extent = {{-8, -8}, {8, 8}})));
  inner Modelica.Blocks.Noise.GlobalSeed globalSeed(fixedSeed = seed, enableNoise = sigma_turb > 0) annotation(
    Placement(transformation(origin = {-70, -80}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Noise.NormalNoise noise[3](each samplePeriod = Ts_noise, each mu = 0,
    sigma = sigma_white*{1, 1, 0.5}, fixedLocalSeed = {11, 22, 33}, each useAutomaticLocalSeed = false) annotation(
    Placement(transformation(origin = {-70, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.FirstOrder filter[3](each T = T_turb, each k = 1, each initType = Modelica.Blocks.Types.Init.InitialOutput, each y_start = 0) annotation(
    Placement(transformation(origin = {-30, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add addX annotation(
    Placement(transformation(origin = {60, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add addY annotation(
    Placement(transformation(origin = {60, 10}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(mean.y, speed.u1) annotation(
    Line(points = {{-59, 70}, {-50, 70}, {-50, 56}, {-42, 56}}, color = {0, 0, 127}));
  connect(gust.y, speed.u2) annotation(
    Line(points = {{-59, 30}, {-50, 30}, {-50, 44}, {-42, 44}}, color = {0, 0, 127}));
  connect(speed.y, toX.u) annotation(
    Line(points = {{-19, 50}, {-10, 50}, {-10, 70}, {0, 70}}, color = {0, 0, 127}));
  connect(speed.y, toY.u) annotation(
    Line(points = {{-19, 50}, {-10, 50}, {-10, 30}, {0, 30}}, color = {0, 0, 127}));
  connect(noise.y, filter.u) annotation(
    Line(points = {{-59, -30}, {-42, -30}}, color = {0, 0, 127}, thickness = 0.5));
  connect(toX.y, addX.u1) annotation(
    Line(points = {{19, 70}, {36, 70}, {36, 56}, {48, 56}}, color = {0, 0, 127}));
  connect(filter[1].y, addX.u2) annotation(
    Line(points = {{-19, -30}, {30, -30}, {30, 44}, {48, 44}}, color = {0, 0, 127}));
  connect(toY.y, addY.u1) annotation(
    Line(points = {{19, 30}, {36, 30}, {36, 16}, {48, 16}}, color = {0, 0, 127}));
  connect(filter[2].y, addY.u2) annotation(
    Line(points = {{-19, -30}, {30, -30}, {30, 4}, {48, 4}}, color = {0, 0, 127}));
  connect(addX.y, v_wind[1]) annotation(
    Line(points = {{71, 50}, {90, 50}, {90, 0}, {110, 0}}, color = {0, 0, 127}));
  connect(addY.y, v_wind[2]) annotation(
    Line(points = {{71, 10}, {90, 10}, {90, 0}, {110, 0}}, color = {0, 0, 127}));
  connect(filter[3].y, v_wind[3]) annotation(
    Line(points = {{-19, -30}, {90, -30}, {90, 0}, {110, 0}}, color = {0, 0, 127}));
  annotation(
    Icon(graphics = {
      Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 127}, fillColor = {235, 245, 255}, fillPattern = FillPattern.Solid),
      Line(points = {{-80, 40}, {-20, 40}, {0, 50}, {20, 40}, {60, 40}}, color = {0, 100, 200}, thickness = 1),
      Line(points = {{-80, 0}, {-30, 0}, {-10, -10}, {10, 0}, {70, 0}}, color = {0, 100, 200}, thickness = 1),
      Line(points = {{-80, -40}, {-40, -40}, {-20, -30}, {0, -40}, {50, -40}}, color = {0, 100, 200}, thickness = 1),
      Text(extent = {{-100, 90}, {100, 60}}, textString = "wind"),
      Text(extent = {{-150, -105}, {150, -135}}, textString = "%name", textColor = {0, 0, 255})}));
end WindProfile;
