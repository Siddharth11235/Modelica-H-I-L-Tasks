within InterProcessCommunication.Examples.InterProcessExamples;

model SIMO_test
Real ModelicaInput (start = 200, fixed = true );

  Real ModelicaOutput1 (start = 100,fixed = true );
  Real ModelicaOutput2 (start = 200,fixed = true );
  Real OutputDummy1;
  Real OutputDummy2;

 parameter Real sampleTime = 0.02; 

  // ModelicaInput = InterProcessCommunication.SharedMemory.SharedMemoryRead(1);
 
 equation
    
    ModelicaOutput1 = 100;
    ModelicaOutput2 = 200;
    when sample(0, sampleTime) then
        OutputDummy1 = InterProcessCommunication.SharedMemory.SharedMemoryWrite(1, ModelicaOutput1); 
      OutputDummy2 = InterProcessCommunication.SharedMemory.SharedMemoryWrite(2, ModelicaOutput2); 
      ModelicaInput = InterProcessCommunication.SharedMemory.SharedMemoryRead(1);
       
    end when;
   
annotation(
    experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-6, Interval = 0.002));
end SIMO_test;