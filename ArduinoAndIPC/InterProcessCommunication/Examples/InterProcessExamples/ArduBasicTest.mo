within InterProcessCommunication.Examples.InterProcessExamples;

model ArduBasicTest
  Real ModelicaInput;
  Real ModelicaOutput (start = 1);
  Real OutputDummy;
     equation
    
    ModelicaOutput = 100*sin(time);
    when sample(0, 0.05) then
     
      ModelicaInput = InterProcessCommunication.SharedMemory.SharedMemoryRead(1);
      OutputDummy = InterProcessCommunication.SharedMemory.SharedMemoryWrite(1, ModelicaOutput);   
    end when;
annotation(
    experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-6, Interval = 0.05));
end ArduBasicTest;