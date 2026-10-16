within ModeloDrone.SubSistemas;

model MotorPowerSimple
  parameter Real kPower(unit="W/(rad/s)^3") = 1e-8
    "Coeficiente aproximado de potencia";

  Modelica.Blocks.Interfaces.RealInput w(unit="rad/s")
    "Velocidad del rotor";

  Modelica.Blocks.Interfaces.RealOutput P(unit="W")
    "Potencia eléctrica estimada";

equation
  P = kPower*max(w, 0)^3;

end MotorPowerSimple;
