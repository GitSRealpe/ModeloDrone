within ModeloDrone.SubSistemas;

block ModeloESC "ESC simplificado: convierte señal de throttle en voltaje PWM"
  import Modelica.Blocks.Interfaces.RealOutput;
  import Modelica.Blocks.Interfaces.RealInput;
  import SI = Modelica.Units.SI;

  parameter Real w_max(unit = "rad/s") = 1000 "Velocidad máxima del motor [rad/s]";
  parameter Real eficiencia = 0.95 "Eficiencia del ESC";
  parameter Real I_max = 10.0 "Corriente máxima del ESC [A]";

  RealInput w_cmd(unit = "rad/s") "Comando de velocidad del motor [rad/s]" annotation(
    Placement(transformation(origin = {0, -76}, extent = {{-140, 55}, {-100, 95}})));
  RealInput V_battery(unit = "V") "Voltaje de la batería [V]" annotation(
    Placement(transformation(origin = {0, -20}, extent = {{-140, 55}, {-100, 95}})));

  RealOutput V_out(unit = "V") "Voltaje entregado al motor [V]" annotation(
    Placement(transformation(origin = {240, -76}, extent = {{-140, 55}, {-100, 95}})));
  RealOutput I_entrada(unit = "A") "Corriente extraída de la batería [A]" annotation(
    Placement(transformation(origin = {240, -20}, extent = {{-140, 55}, {-100, 95}})));

protected
  Real duty_cycle "Ciclo de trabajo del PWM [0..1]";

equation
  duty_cycle = min(1.0, max(0.0, w_cmd / w_max));
  V_out = V_battery * duty_cycle * eficiencia;
  I_entrada = I_max * duty_cycle^2;

  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {
      Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}),
      Text(textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"),
      Text(origin = {-90, 4}, textColor = {0, 0, 127}, extent = {{5, 20}, {95, -20}}, textString = "ESC")
    }),
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})));
end ModeloESC;
