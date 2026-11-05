within DroneSimulation.finalSubsystems;

model DronePlant
  import SI = Modelica.Units.SI;
  parameter Real Q_bat_Ah = 1.0 annotation(Dialog(group = "Battery"));
  parameter Integer N_cells = 2 annotation(Dialog(group = "Battery"));
  parameter Real specificEnergy(unit = "W.h/kg") = 150 annotation(Dialog(group = "Battery"));
  parameter Real R_times_Ah(unit = "Ohm.A.h") = 0.015 annotation(Dialog(group = "Battery"));
  parameter SI.Voltage V_cell_nominal = 3.7 annotation(Dialog(group = "Battery"));
  parameter SI.Voltage V_cell_cutoff = 3.3 annotation(Dialog(group = "Battery"));
  final parameter Real E_bat_Wh = N_cells*Q_bat_Ah*V_cell_nominal;
  final parameter SI.Mass m_battery(displayUnit = "g") = E_bat_Wh/specificEnergy;
  final parameter SI.Resistance R_bat = R_times_Ah/Q_bat_Ah;
  parameter SI.Length D_prop = 0.127 annotation(Dialog(group = "Propeller"));
  parameter Real C_T = 0.10 annotation(Dialog(group = "Propeller"));
  parameter Real C_P = 0.05 annotation(Dialog(group = "Propeller"));
  parameter SI.Density rho_air = 1.225 annotation(Dialog(group = "Propeller"));
  parameter SI.Mass m_blade = 0.002 annotation(Dialog(group = "Propeller"));
  final parameter Real kf(unit = "N.s2/rad2") = C_T*rho_air*D_prop^4/(4*Modelica.Constants.pi^2);
  final parameter Real km(unit = "N.m.s2/rad2") = C_P*rho_air*D_prop^5/(8*Modelica.Constants.pi^3);
  final parameter SI.Inertia Jp = 2*m_blade*(D_prop/2)^2/3;
  parameter Real KV(unit = "rpm/V") = 2300 annotation(Dialog(group = "Motor"));
  parameter SI.Resistance Ra = 0.15 annotation(Dialog(group = "Motor"));
  parameter SI.Power P_fric = 0.5 annotation(Dialog(group = "Motor"));
  parameter SI.AngularVelocity w_fric = 800 annotation(Dialog(group = "Motor"));
  final parameter Real psi(unit = "V.s/rad") = 60/(2*Modelica.Constants.pi*KV);
  final parameter SI.Voltage VaNominal = N_cells*V_cell_nominal;
  final parameter SI.Current IaNominal = 5;
  final parameter SI.AngularVelocity wNominal = (VaNominal - Ra*IaNominal)/psi;
  parameter SI.Length l = 0.2 annotation(Dialog(group = "Airframe"));
  parameter SI.Mass m_frame(displayUnit = "g") = 0.2 annotation(Dialog(group = "Airframe"));
  parameter SI.Length cog_offset = 0.02 annotation(Dialog(group = "Airframe"));
  final parameter SI.Mass mass(displayUnit = "g") = m_frame + m_battery;
  final parameter SI.Mass m_blades = 2*m_blade;
  final parameter SI.AngularVelocity w_hover = sqrt(mass*Modelica.Constants.g_n/(4*kf));
  final parameter SI.Current ia_hover = (km*w_hover^2 + P_fric/w_fric*(w_hover/w_fric))/psi;
  parameter Modelica.Electrical.Batteries.ParameterRecords.TransientData.CellData cellData(
    Qnom = Q_bat_Ah*3600, OCVmax = 4.2, OCVmin = 3.0, useLinearSOCDependency = false,
    OCV_SOC = [0, 3.0/4.2; 0.05, 3.45/4.2; 0.1, 3.6/4.2; 0.2, 3.7/4.2; 0.5, 3.8/4.2; 0.8, 3.95/4.2; 1, 1],
    Ri = R_bat, nRC = 1,
    rcData = {Modelica.Electrical.Batteries.ParameterRecords.TransientData.RCData(R = 0.2*R_bat, C = 10/(0.2*R_bat))}) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Interfaces.RealInput duty[4] annotation(
    Placement(transformation(extent = {{-140, -20}, {-100, 20}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b airframe annotation(
    Placement(transformation(origin = {-10, -44}, extent = {{84, -16}, {116, 16}}), iconTransformation(extent = {{84, -16}, {116, 16}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin bat_p annotation(
    Placement(transformation(origin = {-96, -68}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-40, -100}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin bat_n annotation(
    Placement(transformation(origin = {-96, -48}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {40, -100}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Electrical.Batteries.BatteryStacks.CellRCStack battery(Ns = N_cells, Np = 1, cellData = cellData, SOC(fixed = true, start = 0.999)) annotation(
    Placement(transformation(origin = {-60, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(
    Placement(transformation(origin = {-60, -90}, extent = {{-8, -8}, {8, 8}})));
  ESC esc1 annotation(
    Placement(transformation(origin = {-30, -45}, extent = {{-8, -8}, {8, 8}})));
  ESC esc2 annotation(
    Placement(transformation(origin = {-30, -15}, extent = {{-8, -8}, {8, 8}})));
  ESC esc3 annotation(
    Placement(transformation(origin = {-30, 15}, extent = {{-8, -8}, {8, 8}})));
  ESC esc4 annotation(
    Placement(transformation(origin = {-30, 45}, extent = {{-8, -8}, {8, 8}})));
  ElectricPropeller prop1(dir = 1, kf = kf, km = km, Jp = Jp, m_blade = m_blade, r_blade = D_prop/2, VaNominal = VaNominal, IaNominal = IaNominal, wNominal = wNominal, Ra = Ra, P_fric = P_fric, w_fric = w_fric, fric_exp = 1, w_start = w_hover, ia_start = ia_hover) annotation(
    Placement(transformation(origin = {10, -45}, extent = {{-8, -8}, {8, 8}})));
  ElectricPropeller prop2(dir = -1, kf = kf, km = km, Jp = Jp, m_blade = m_blade, r_blade = D_prop/2, VaNominal = VaNominal, IaNominal = IaNominal, wNominal = wNominal, Ra = Ra, P_fric = P_fric, w_fric = w_fric, fric_exp = 1, w_start = w_hover, ia_start = ia_hover) annotation(
    Placement(transformation(origin = {10, -15}, extent = {{-8, -8}, {8, 8}})));
  ElectricPropeller prop3(dir = 1, kf = kf, km = km, Jp = Jp, m_blade = m_blade, r_blade = D_prop/2, VaNominal = VaNominal, IaNominal = IaNominal, wNominal = wNominal, Ra = Ra, P_fric = P_fric, w_fric = w_fric, fric_exp = 1, w_start = w_hover, ia_start = ia_hover) annotation(
    Placement(transformation(origin = {10, 15}, extent = {{-8, -8}, {8, 8}})));
  ElectricPropeller prop4(dir = -1, kf = kf, km = km, Jp = Jp, m_blade = m_blade, r_blade = D_prop/2, VaNominal = VaNominal, IaNominal = IaNominal, wNominal = wNominal, Ra = Ra, P_fric = P_fric, w_fric = w_fric, fric_exp = 1, w_start = w_hover, ia_start = ia_hover) annotation(
    Placement(transformation(origin = {10, 45}, extent = {{-8, -8}, {8, 8}})));
  DroneLibrary.Mechanical.Chassis.Variants.droneChassis chassis(
    m = (m_frame - 4*m_blades)/5,
    length = cog_offset,
    bodyShape(r = {0, l, 0}, r_CM = {0, l/2, 0}),
    bodyShape1(r = {l, 0, 0}, r_CM = {l/2, 0, 0}),
    bodyShape2(r = {0, -l, 0}, r_CM = {0, -l/2, 0}),
    bodyShape3(r = {-l, 0, 0}, r_CM = {-l/2, 0, 0}), animation = false) annotation(
    Placement(transformation(origin = {60, 0}, extent = {{-25, -15}, {25, 15}})));
  Modelica.Mechanics.MultiBody.Parts.Body batteryBody(m = m_battery, r_CM = {0, 0, -0.01}, I_11 = 1e-6, I_22 = 1e-6, I_33 = 1e-6, sphereDiameter = 0.02, sphereColor = {0, 0, 255}) annotation(
    Placement(transformation(origin = {60, -50}, extent = {{-8, -8}, {8, 8}}, rotation = 270)));
equation
  connect(duty[1], esc1.duty) annotation(
    Line(points = {{-120, 0}, {-80, 0}, {-80, -30}, {-30, -30}, {-30, -35}}, color = {0, 0, 127}));
  connect(duty[2], esc2.duty) annotation(
    Line(points = {{-120, 0}, {-80, 0}, {-80, 0}, {-30, 0}, {-30, -5}}, color = {0, 0, 127}));
  connect(duty[3], esc3.duty) annotation(
    Line(points = {{-120, 0}, {-80, 0}, {-80, 30}, {-30, 30}, {-30, 25}}, color = {0, 0, 127}));
  connect(duty[4], esc4.duty) annotation(
    Line(points = {{-120, 0}, {-80, 0}, {-80, 60}, {-30, 60}, {-30, 55}}, color = {0, 0, 127}));
  connect(battery.n, ground.p) annotation(
    Line(points = {{-60, -48}, {-60, -82}}, color = {0, 0, 255}));
  connect(battery.p, esc1.bat_p) annotation(
    Line(points = {{-60, -68}, {-60, -40}, {-38, -40}}, color = {0, 0, 255}));
  connect(battery.p, esc2.bat_p) annotation(
    Line(points = {{-60, -68}, {-60, -10}, {-38, -10}}, color = {0, 0, 255}));
  connect(battery.p, esc3.bat_p) annotation(
    Line(points = {{-60, -68}, {-60, 20}, {-38, 20}}, color = {0, 0, 255}));
  connect(battery.p, esc4.bat_p) annotation(
    Line(points = {{-60, -68}, {-60, 50}, {-38, 50}}, color = {0, 0, 255}));
  connect(battery.n, esc1.bat_n) annotation(
    Line(points = {{-60, -48}, {-52, -48}, {-52, -50}, {-38, -50}}, color = {0, 0, 255}));
  connect(battery.n, esc2.bat_n) annotation(
    Line(points = {{-60, -48}, {-44, -48}, {-44, -20}, {-38, -20}}, color = {0, 0, 255}));
  connect(battery.n, esc3.bat_n) annotation(
    Line(points = {{-60, -48}, {-44, -48}, {-44, 10}, {-38, 10}}, color = {0, 0, 255}));
  connect(battery.n, esc4.bat_n) annotation(
    Line(points = {{-60, -48}, {-44, -48}, {-44, 40}, {-38, 40}}, color = {0, 0, 255}));
  connect(battery.p, bat_p) annotation(
    Line(points = {{-60, -68}, {-96, -68}}, color = {0, 0, 255}));
  connect(battery.n, bat_n) annotation(
    Line(points = {{-60, -48}, {-96, -48}}, color = {0, 0, 255}));
  connect(esc1.mot_p, prop1.p) annotation(
    Line(points = {{-22, -40}, {2, -40}}, color = {0, 0, 255}));
  connect(esc1.mot_n, prop1.n) annotation(
    Line(points = {{-22, -50}, {2, -50}}, color = {0, 0, 255}));
  connect(esc2.mot_p, prop2.p) annotation(
    Line(points = {{-22, -10}, {2, -10}}, color = {0, 0, 255}));
  connect(esc2.mot_n, prop2.n) annotation(
    Line(points = {{-22, -20}, {2, -20}}, color = {0, 0, 255}));
  connect(esc3.mot_p, prop3.p) annotation(
    Line(points = {{-22, 20}, {2, 20}}, color = {0, 0, 255}));
  connect(esc3.mot_n, prop3.n) annotation(
    Line(points = {{-22, 10}, {2, 10}}, color = {0, 0, 255}));
  connect(esc4.mot_p, prop4.p) annotation(
    Line(points = {{-22, 50}, {2, 50}}, color = {0, 0, 255}));
  connect(esc4.mot_n, prop4.n) annotation(
    Line(points = {{-22, 40}, {2, 40}}, color = {0, 0, 255}));
  connect(prop1.Airframe, chassis.frame_a3) annotation(
    Line(points = {{18, -45}, {30, -45}, {30, -8}, {35, -8}}, color = {95, 95, 95}));
  connect(prop2.Airframe, chassis.frame_a2) annotation(
    Line(points = {{18, -15}, {28, -15}, {28, -3}, {35, -3}}, color = {95, 95, 95}));
  connect(prop3.Airframe, chassis.frame_a) annotation(
    Line(points = {{18, 15}, {28, 15}, {28, 3}, {35, 3}}, color = {95, 95, 95}));
  connect(prop4.Airframe, chassis.frame_a1) annotation(
    Line(points = {{18, 45}, {30, 45}, {30, 8}, {35, 8}}, color = {95, 95, 95}));
  connect(chassis.frame_a4, batteryBody.frame_a) annotation(
    Line(points = {{60, -15}, {60, -42}}, color = {95, 95, 95}));
  connect(chassis.frame_a4, airframe) annotation(
    Line(points = {{60, -15}, {60, -25}, {90, -25}, {90, -44}}, color = {95, 95, 95}));
  annotation(
    Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {200, 0, 0}, fillColor = {255, 245, 245}, fillPattern = FillPattern.Solid, radius = 10), Bitmap(extent = {{-95, 95}, {95, -95}}, fileName = "modelica://DroneSimulation/finalSubsystems/Resources/icon_plant.png"), Text(extent = {{-150, 140}, {150, 105}}, textString = "%name", textColor = {200, 0, 0}), Text(extent = {{-98, 30}, {-60, 10}}, textString = "d", textColor = {0, 0, 127})}),
  experiment(StartTime = 0, StopTime = 50, Tolerance = 1e-06, Interval = 0.01));
end DronePlant;