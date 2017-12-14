within MDD_Practice;
class avr_io
  
  inner Modelica_DeviceDrivers.EmbeddedTargets.AVR.Blocks.Microcontroller mcu(platform = Modelica_DeviceDrivers.EmbeddedTargets.AVR.Types.Platform.ATmega328P)  annotation(
    Placement(visible = true, transformation(origin = {-67, 43}, extent = {{-17, -17}, {17, 17}}, rotation = 0)));
  Modelica_DeviceDrivers.EmbeddedTargets.AVR.Blocks.ADC adc(analogPort = Modelica_DeviceDrivers.EmbeddedTargets.AVR.Types.AnalogPort.A5, voltageReference = 5, voltageReferenceSelect = Modelica_DeviceDrivers.EmbeddedTargets.AVR.Types.VRefSelect.AREF)  annotation(
    Placement(visible = true, transformation(origin = {-38, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain1 annotation(
    Placement(visible = true, transformation(origin = {12, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(adc.y, gain1.u) annotation(
    Line(points = {{-26, -2}, {-2, -2}, {-2, 0}, {0, 0}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica_DeviceDrivers(version = "1.5.0"), Modelica(version = "3.2.2")));
end avr_io;