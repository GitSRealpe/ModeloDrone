within DroneLibrary.Mechanical.Rotor.Variants;
model rotor
  extends DroneLibrary.Mechanical.Rotor.Templates.rotor;

  Modelica.Mechanics.MultiBody.Joints.Revolute revolute(animation=false)
    annotation (Placement(transformation(extent={{4,14},{24,34}})));

  Modelica.Mechanics.MultiBody.Sensors.RelativeAngularVelocity
    relativeAngularVelocity
    annotation (Placement(transformation(extent={{30,74},{50,94}})));
  Modelica.Mechanics.MultiBody.Forces.Torque torque( animation=false)
    annotation (Placement(transformation(extent={{10,-74},{30,-54}})));
  Modelica.Blocks.Math.Gain gain(k=-d_bearing) "Bearing friction between airframe and blade" annotation (Placement(
        transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={40,0})));
  Blocks.Routing.RealExtract realExtract annotation (Placement(
        transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={40,40})));
  Blocks.Routing.RealExtend realExtend2 annotation (Placement(
        transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={40,-22})));

  Modelica.Blocks.Sources.RealExpression realExpression(y=-kd*
        relativeAngularVelocity.w_rel[3]*abs(relativeAngularVelocity.w_rel[3]))
    "Aerodynamic drag torque on the blade, opposite to its rotation"
    annotation (Placement(transformation(extent={{-96,-90},{-76,-70}})));
  Blocks.Routing.RealExtend realExtend1 annotation (Placement(
        transformation(
        extent={{-6,-6},{6,6}},
        origin={-40,-80})));
  Modelica.Mechanics.MultiBody.Forces.WorldTorque dragTorque(animation=false,
      resolveInFrame=Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b)
    "Air acts on the blade only (no reaction on the airframe)"
    annotation (Placement(transformation(extent={{-22,-90},{-2,-70}})));
  parameter Real k=-1 "Gain value multiplied with input signal";
  parameter Real kd(unit="N.m.s2/rad2")=3.5e-6 "Aerodynamic drag coefficient of the blade: tau = -kd*w*|w|";
  parameter Real d_bearing(unit="N.m.s/rad")=0 "Viscous bearing friction between airframe and blade";
equation
  connect(relativeAngularVelocity.w_rel, realExtract.u)
    annotation (Line(points={{40,73},{40,46}}, color={0,0,127}));
  connect(realExtend2.y, torque.torque) annotation (Line(points={{40,-28.6},{40,
          -40},{14,-40},{14,-52}}, color={0,0,127}));
  connect(relativeAngularVelocity.frame_a, torque.frame_a) annotation (Line(
      points={{30,84},{-14,84},{-14,-64},{10,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute.frame_a, torque.frame_a) annotation (Line(
      points={{4,24},{-14,24},{-14,-64},{10,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(revolute.frame_b, torque.frame_b) annotation (Line(
      points={{24,24},{62,24},{62,-64},{30,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(relativeAngularVelocity.frame_b, torque.frame_b) annotation (Line(
      points={{50,84},{62,84},{62,-64},{30,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(realExtend2.u, gain.y)
    annotation (Line(points={{40,-14.8},{40,-6.6}}, color={0,0,127}));
  connect(realExtend1.y, dragTorque.torque)
    annotation (Line(points={{-33.4,-80},{-24,-80}}, color={0,0,127}));
  connect(dragTorque.frame_b, torque.frame_b) annotation (Line(
      points={{-2,-80},{40,-80},{40,-64},{30,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(realExtract.y, gain.u)
    annotation (Line(points={{40,33.4},{40,7.2}}, color={0,0,127}));
  connect(realExpression.y, realExtend1.u)
    annotation (Line(points={{-75,-80},{-47.2,-80}}, color={0,0,127}));
  connect(torque_1, torque.frame_a) annotation (Line(
      points={{-102,60},{-14,60},{-14,-64},{10,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(torque_2, torque.frame_b) annotation (Line(
      points={{-102,0},{62,0},{62,-64},{30,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(force, torque.frame_a) annotation (Line(
      points={{-102,-62},{-58,-62},{-58,24},{-14,24},{-14,-64},{10,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(Airframe, torque.frame_a) annotation (Line(
      points={{102,52},{76,52},{76,98},{-14,98},{-14,-64},{10,-64}},
      color={95,95,95},
      thickness=0.5));
  connect(Blade, torque.frame_b) annotation (Line(
      points={{102,-40},{62,-40},{62,-64},{30,-64}},
      color={95,95,95},
      thickness=0.5));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}),
          Text(
          extent={{-72,22},{76,-20}},
          lineColor={28,108,200},
          textString="Rotor")}));
end rotor;
