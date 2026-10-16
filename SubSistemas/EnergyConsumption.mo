within ModeloDrone.SubSistemas;

model EnergyConsumption
  Modelica.Blocks.Interfaces.RealInput P(unit="W")
    "Potencia total consumida";

  Modelica.Blocks.Interfaces.RealOutput E_Wh(unit="W.h")
    "Energia consumida";

  Real E_J(unit="J", start=0, fixed=true)
    "Energia consumida en julios";

equation
  der(E_J) = max(P, 0);
  E_Wh = E_J/3600;

end EnergyConsumption;
