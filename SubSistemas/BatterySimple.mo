within ModeloDrone.SubSistemas;

model BatterySimple
  import SI = Modelica.Units.SI;

  parameter Real capacity_Wh(unit="W.h") = 20
    "Capacidad nominal de la bateria";

  parameter Real initialSOC = 1
    "Estado de carga inicial: 1 = 100%";

  Modelica.Blocks.Interfaces.RealInput P(unit="W")
    "Potencia consumida";

  Modelica.Blocks.Interfaces.RealOutput SOC
    "Estado de carga entre 0 y 1";

  Modelica.Blocks.Interfaces.RealOutput remainingEnergy(unit="W.h")
    "Energia restante";

protected
  Real consumedEnergy(unit="W.h", start=0, fixed=true);

equation
  der(consumedEnergy) = max(P, 0)/3600;
  SOC = max(0, min(1, initialSOC - consumedEnergy/capacity_Wh));
  remainingEnergy = SOC*capacity_Wh;

end BatterySimple;
