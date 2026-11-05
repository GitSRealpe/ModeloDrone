within DroneSimulation.finalSubsystems;

model Disturbances
  import SI = Modelica.Units.SI;
  parameter SI.Mass m_payload(displayUnit = "g") = 0 annotation(Dialog(group = "Payload"));
  parameter SI.Length hook_length = 0.05 annotation(Dialog(group = "Payload"));
  parameter SI.Velocity V_wind = 3 annotation(Dialog(group = "Wind"));
  parameter Real wind_dir_deg = 0 annotation(Dialog(group = "Wind"));
  parameter SI.Velocity V_gust = 3 annotation(Dialog(group = "Wind"));
  parameter SI.Time t_gust = 60 annotation(Dialog(group = "Wind"));
  parameter SI.Velocity sigma_turb = 0.5 annotation(Dialog(group = "Wind"));
  parameter Integer seed = 1 annotation(Dialog(group = "Wind"));
  parameter SI.Area CdA_h = 0.015 annotation(Dialog(group = "Drag"));
  parameter SI.Area CdA_v = 0.04 annotation(Dialog(group = "Drag"));
  parameter SI.Density rho_air = 1.225 annotation(Dialog(group = "Drag"));

  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(extent = {{84, -16}, {116, 16}})));

  WindProfile wind(V_mean = V_wind, direction_deg = wind_dir_deg, V_gust = V_gust, t_gust = t_gust, sigma_turb = sigma_turb, seed = seed) annotation(
    Placement(transformation(origin = {-50, 40}, extent = {{-15, -15}, {15, 15}})));
  AeroDrag drag(CdA_h = CdA_h, CdA_v = CdA_v, rho = rho_air) annotation(
    Placement(transformation(origin = {20, 40}, extent = {{-15, -15}, {15, 15}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation hook(r = {0, 0, -hook_length}, animation = false) annotation(
    Placement(transformation(origin = {40, -40}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body payload(m = m_payload, r_CM = {0, 0, 0}, I_11 = 1e-6, I_22 = 1e-6, I_33 = 1e-6, sphereDiameter = 0.04) annotation(
    Placement(transformation(origin = {0, -40}, extent = {{10, -10}, {-10, 10}})));
equation
  connect(wind.v_wind, drag.v_wind) annotation(
    Line(points = {{-33.5, 40}, {2, 40}}, color = {0, 0, 127}, thickness = 0.5));
  connect(drag.frame_a, frame_a) annotation(
    Line(points = {{35, 40}, {70, 40}, {70, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(frame_a, hook.frame_a) annotation(
    Line(points = {{100, 0}, {70, 0}, {70, -40}, {50, -40}}, color = {95, 95, 95}));
  connect(hook.frame_b, payload.frame_a) annotation(
    Line(points = {{30, -40}, {10, -40}}, color = {95, 95, 95}));
  annotation(
    Icon(graphics = {
      Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 100, 200}, fillColor = {240, 248, 255}, fillPattern = FillPattern.Solid, radius = 10),
      Bitmap(extent = {{-90, 90}, {90, -90}}, fileName = "modelica://DroneSimulation/finalSubsystems/Resources/icon_disturbances.png"),
      Text(extent = {{-150, 140}, {150, 105}}, textString = "%name", textColor = {0, 100, 200})}));
end Disturbances;
