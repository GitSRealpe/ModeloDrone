within ModeloDrone.SubSistemas;

model RotorMotorSimple
  import SI = Modelica.Units.SI;

  parameter SI.AngularVelocity wMax = 1000
    "Velocidad maxima del rotor";

  parameter SI.Time tauMotor = 0.1
    "Constante de tiempo del motor";

  Modelica.Blocks.Interfaces.RealInput wCmd(
    unit = "rad/s") "Velocidad solicitada";

  Modelica.Blocks.Interfaces.RealOutput w(
    unit = "rad/s") "Velocidad real del rotor";

equation
  der(w) = (min(max(wCmd, 0), wMax) - w)/tauMotor;

initial equation
  w = 0;

end RotorMotorSimple;
