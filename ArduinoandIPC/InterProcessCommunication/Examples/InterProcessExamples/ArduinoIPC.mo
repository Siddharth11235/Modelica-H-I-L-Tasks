within InterProcessCommunication.Examples.InterProcessExamples;

model ArduinoIPC
Real ModelicaInput (start = 0,fixed = true );
  Real ModelicaOutput (start = 0,fixed = true );
  Real OutputDummy;
 Real OP;    
 parameter Real sampleTime = 0.01; 
 initial equation
  OutputDummy = InterProcessCommunication.SharedMemory.SharedMemoryWrite(1, ModelicaOutput); 
  // ModelicaInput = InterProcessCommunication.SharedMemory.SharedMemoryRead(1);
 
 equation
    
    ModelicaOutput = time;
    when sample(0, sampleTime) then
        OutputDummy = InterProcessCommunication.SharedMemory.SharedMemoryWrite(1, ModelicaOutput); 

      ModelicaInput = InterProcessCommunication.SharedMemory.SharedMemoryRead(1);
       
    end when;
   OP =  ModelicaOutput - ModelicaInput;
annotation(
    experiment(StartTime = 0, StopTime = 60, Tolerance = 1e-6, Interval = 0.002));
end ArduinoIPC;