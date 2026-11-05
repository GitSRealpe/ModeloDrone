within DroneSimulation.finalSubsystems;

model ElectricPropeller
  import SI = Modelica.Units.SI;
  parameter Real dir = 1;
  parameter Real kf(unit = "N.s2/rad2") = 1e-5;
  parameter Real km(unit = "N.m.s2/rad2") = 1e-7;
  parameter SI.Inertia Jp = 2e-6;
  parameter SI.Mass m_blade = 0.01;
  parameter SI.Length r_blade = 0.154;
  parameter SI.Voltage VaNominal = 3.7 annotation(Dialog(group = "Motor"));
  parameter SI.Current IaNominal = 1 annotation(Dialog(group = "Motor"));
  parameter SI.AngularVelocity wNominal = 360 annotation(Dialog(group = "Motor"));
  parameter SI.Resistance Ra = 0.1 annotation(Dialog(group = "Motor"));
  parameter SI.Inductance La = 1e-4 annotation(Dialog(group = "Motor"));
  parameter SI.Inertia Jr = 1e-6 annotation(Dialog(group = "Motor"));
  parameter SI.Power P_fric = 0 annotation(Dialog(group = "Motor"));
  parameter SI.AngularVelocity w_fric = wNominal annotation(Dialog(group = "Motor"));
  parameter Real fric_exp = 1 annotation(Dialog(group = "Motor"));
  parameter SI.AngularVelocity w_start = 0 annotation(Dialog(group = "Initialization"));
  parameter SI.Current ia_start = 0 annotation(Dialog(group = "Initialization"));
  parameter Boolean animation = true;

  Modelica.Mechanics.MultiBody.Interfaces.Frame_a Airframe annotation(
    Placement(transformation(extent = {{84, -16}, {116, 16}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p annotation(
    Placement(transformation(extent = {{-110, 50}, {-90, 70}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n annotation(
    Placement(transformation(extent = {{-110, -70}, {-90, -50}})));

  Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet dcpm(
    VaNominal = VaNominal, IaNominal = IaNominal, wNominal = wNominal,
    TaNominal = 293.15, TaRef = 293.15, TaOperational = 293.15, alpha20a = 0,
    Ra = Ra, La = La, Jr = Jr, Js = Jr, useSupport = true,
    frictionParameters(PRef = P_fric, wRef = w_fric, power_w = fric_exp),
    ia(start = ia_start, fixed = true)) annotation(
    Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute(
    n = {0, 0, dir}, useAxisFlange = true, animation = false,
    phi(start = 0, fixed = true), w(start = w_start, fixed = true)) annotation(
    Placement(transformation(origin = {40, 0}, extent = {{10, -10}, {-10, 10}})));
  DroneLibrary.Mechanical.Blades.Variants.Blades blades(animation = animation,
    bodyShape4(m = m_blade, r = {r_blade, 0, 0}, r_CM = {0, 0, 0}, I_11 = Jp/2, I_22 = Jp/2, I_33 = Jp/2),
    bodyShape5(m = m_blade, r = {-r_blade, 0, 0}, r_CM = {0, 0, 0}, I_11 = Jp/2, I_22 = Jp/2, I_33 = Jp/2)) annotation(
    Placement(transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Sensors.RelSpeedSensor relSpeed annotation(
    Placement(transformation(origin = {40, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Math.Product w2 annotation(
    Placement(transformation(origin = {0, -60}, extent = {{-6, -6}, {6, 6}}, rotation = 180)));
  Modelica.Blocks.Math.Gain thrustGain(k = kf) annotation(
    Placement(transformation(origin = {-24, -60}, extent = {{-6, -6}, {6, 6}}, rotation = 180)));
  DroneLibrary.Blocks.Routing.RealExtend thrustVec annotation(
    Placement(transformation(origin = {-48, -60}, extent = {{-6, -6}, {6, 6}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce thrust(animation = false, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {-74, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
  Modelica.Blocks.Math.Abs absW annotation(
    Placement(transformation(origin = {0, -86}, extent = {{-6, -6}, {6, 6}}, rotation = 180)));
  Modelica.Blocks.Math.Product wAbsW annotation(
    Placement(transformation(origin = {-24, -86}, extent = {{-6, -6}, {6, 6}}, rotation = 180)));
  Modelica.Blocks.Math.Gain dragGain(k = -km*dir) annotation(
    Placement(transformation(origin = {-48, -86}, extent = {{-6, -6}, {6, 6}}, rotation = 180)));
  DroneLibrary.Blocks.Routing.RealExtend dragVec annotation(
    Placement(transformation(origin = {-70, -86}, extent = {{-6, -6}, {6, 6}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Forces.WorldTorque drag(animation = false, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(transformation(origin = {-90, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
equation
  connect(p, dcpm.pin_ap) annotation(
    Line(points = {{-100, 60}, {-44, 60}, {-44, 10}}, color = {0, 0, 255}));
  connect(n, dcpm.pin_an) annotation(
    Line(points = {{-100, -60}, {-80, -60}, {-80, 20}, {-56, 20}, {-56, 10}}, color = {0, 0, 255}));
  connect(dcpm.flange, revolute.axis) annotation(
    Line(points = {{-40, 0}, {-20, 0}, {-20, 20}, {40, 20}, {40, 10}}));
  connect(dcpm.support, revolute.support) annotation(
    Line(points = {{-40, -10}, {-20, -10}, {-20, -20}, {46, -20}, {46, 10}}));
  connect(Airframe, revolute.frame_a) annotation(
    Line(points = {{100, 0}, {50, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(revolute.frame_b, blades.Input) annotation(
    Line(points = {{30, 0}, {-12, 0}, {-12, 50}, {-10, 50}}, color = {95, 95, 95}, thickness = 0.5));
  connect(relSpeed.flange_a, revolute.support) annotation(
    Line(points = {{50, -40}, {60, -40}, {60, 14}, {46, 14}, {46, 10}}));
  connect(relSpeed.flange_b, revolute.axis) annotation(
    Line(points = {{30, -40}, {24, -40}, {24, 20}, {40, 20}, {40, 10}}));
  connect(relSpeed.w_rel, w2.u1) annotation(
    Line(points = {{40, -51}, {40, -56}, {7, -56}}, color = {0, 0, 127}));
  connect(relSpeed.w_rel, w2.u2) annotation(
    Line(points = {{40, -51}, {40, -64}, {7, -64}}, color = {0, 0, 127}));
  connect(w2.y, thrustGain.u) annotation(
    Line(points = {{-6.6, -60}, {-16.8, -60}}, color = {0, 0, 127}));
  connect(thrustGain.y, thrustVec.u) annotation(
    Line(points = {{-30.6, -60}, {-40.8, -60}}, color = {0, 0, 127}));
  connect(thrustVec.y, thrust.force) annotation(
    Line(points = {{-54.6, -60}, {-62, -60}}, color = {0, 0, 127}));
  connect(thrust.frame_b, revolute.frame_b) annotation(
    Line(points = {{-84, -60}, {-96, -60}, {-96, -100}, {20, -100}, {20, 0}, {30, 0}}, color = {95, 95, 95}));
  connect(relSpeed.w_rel, absW.u) annotation(
    Line(points = {{40, -51}, {40, -86}, {7, -86}}, color = {0, 0, 127}));
  connect(absW.y, wAbsW.u2) annotation(
    Line(points = {{-6.6, -86}, {-16.8, -86}, {-16.8, -90}}, color = {0, 0, 127}));
  connect(relSpeed.w_rel, wAbsW.u1) annotation(
    Line(points = {{40, -51}, {40, -78}, {-12, -78}, {-12, -82}, {-16.8, -82}}, color = {0, 0, 127}));
  connect(wAbsW.y, dragGain.u) annotation(
    Line(points = {{-30.6, -86}, {-40.8, -86}}, color = {0, 0, 127}));
  connect(dragGain.y, dragVec.u) annotation(
    Line(points = {{-54.6, -86}, {-62.8, -86}}, color = {0, 0, 127}));
  connect(dragVec.y, drag.torque) annotation(
    Line(points = {{-76.6, -86}, {-90, -86}, {-90, -52}}, color = {0, 0, 127}));
  connect(drag.frame_b, revolute.frame_b) annotation(
    Line(points = {{-90, -30}, {-90, -24}, {20, -24}, {20, 0}, {30, 0}}, color = {95, 95, 95}));
  annotation(
    Icon(graphics = {
      Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{-70, 30}, {70, -30}}, lineColor = {0, 0, 0}, fillColor = {200, 200, 200}, fillPattern = FillPattern.Solid),
      Rectangle(extent = {{-10, 30}, {10, -80}}, fillColor = {100, 100, 100}, fillPattern = FillPattern.Solid),
      Text(extent = {{-100, 90}, {100, 50}}, textString = "M"),
      Text(extent = {{-150, -105}, {150, -135}}, textString = "%name", textColor = {0, 0, 255})}));
end ElectricPropeller;
