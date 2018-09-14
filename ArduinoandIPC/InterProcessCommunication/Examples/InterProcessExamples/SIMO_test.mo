within InterProcessCommunication.Examples.InterProcessExamples;

model SIMO_test
Real ModelicaInput (start = 0, fixed = true );

  Real ModelicaOutput1 (start = 1.1,fixed = true );
  Real ModelicaOutput2 (start = 2.2,fixed = true );
  Real ModelicaOutput3 (start = 3.3,fixed = true );

  Real OutputDummy1;
  Real OutputDummy2;
  Real OutputDummy3;
 parameter Real sampleTime = 0.02; 

  // ModelicaInput = InterProcessCommunication.SharedMemory.SharedMemoryRead(1);
 
 equation
    
    ModelicaOutput1 = 1.1;
    ModelicaOutput2 = 2.2;
    ModelicaOutput3 = 3.3;
    when sample(0, sampleTime) then
        OutputDummy1 = InterProcessCommunication.SharedMemory.SharedMemoryWrite(1, ModelicaOutput1); 
      OutputDummy2 = InterProcessCommunication.SharedMemory.SharedMemoryWrite(2, ModelicaOutput2);
      OutputDummy3 = InterProcessCommunication.SharedMemory.SharedMemoryWrite(3, ModelicaOutput3);  

      ModelicaInput = InterProcessCommunication.SharedMemory.SharedMemoryRead(1);
       
    end when;
   
annotation(
    experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-6, Interval = 0.002));
end SIMO_test;