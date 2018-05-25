within InterProcessCommunication.Examples.InterProcessExamples;

model ArduinoIPC
  Real ModelicaInput;
  Real ModelicaOutput (start = 1);
  Real OutputDummy;
  Modelica.Blocks.Continuous.FirstOrder firstOrder1(T = 1, y_start = 1)  annotation(
    Placement(visible = true, transformation(origin = {0, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));  
    
 algorithm
    
    ModelicaOutput := firstOrder1.y;
    when sample(0, 0.05) then
     
      ModelicaInput := InterProcessCommunication.SharedMemory.SharedMemoryRead(1);
      OutputDummy := InterProcessCommunication.SharedMemory.SharedMemoryWrite(1, ModelicaOutput);   
     firstOrder1.u := ModelicaInput;
    end when;
annotation(
    experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-6, Interval = 0.05));end ArduinoIPC;