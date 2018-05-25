within InterProcessCommunication.info;

class Overview
extends Modelica.Icons.Information;
annotation( Documentation(info = "<html>

<p>
<b>Inter Process Communication Library V1.0</b><br /><br />
<b>Aim:</b> To communicate between two OpenModelica Models executing on two separate systems in real time using shared memory and serial communication.
</p>

<p>
<b>Technology Used:</b><br /> 
<u>Software:</u><br /> 
*OpenModelica for simulation.<br /> 
*Shared memory for interprocess data exchange.<br /> 
*RS232 serial communication protocol for inter system data exchange.<br /> 
<u>Hardware:</u><br /> 
*A linux machine<br />
*One serial connection cable.<br />
*Two Arduinos (second optional).<br />
</p>

<p>
<b>Introduction:</b><br />
Library <b>InterProcessCommunication</b> is a Modelica package, which enables Interprocess Communication (IPC) between two different processes using shared memory and Intersystem Communication between two different PC's using serial Communication. To demonstrate the IPC using shared memory, the PID controller model on the arduino and the DCMotor process model on OpenModelica have been considered as two different processes. For inter system communication you can run IPCArdNew.ino package on the arduino and DCMotor package on the PC.
</p>

<p>
The overall objective of the library InterProcessCommunication is to demonstrate the closed loop speed tracking of DC motor using discrete PID controller, with the help of shared memory and serial communication. The <a href=\"modelica://InterProcessCommunication.Examples.InterProcessExamples.DC_Motor_Arduino\">DCMotor</a> model contains the DC motor, along with a speed sensor. The speed sensor measures the speed of the DC motor, which is written into the shared memory. A serial comunication executable Serial_SHM running in background reads this value from shared memory and transfers it to serial port and then to the arduino via serial communication device. The Arduino acquires the measured speed of the DC motor from the shared memory. Based on the difference between setpoint and measured speed, the PID controller flashed on the arduino generates a control signal. This control signal is written into the shared memory, which is acquired by the DCMotor package. The control signal acquired by the DCMotor package, acts as an input to the DC motor. Therefore, the complete closed loop speed tracking of DC motor can be achieved.
</p>


<p>
The data from shared memory goes to the serial port and then to other system and vice versa. This process of reading data from shared memory, sending it to serial port, receiving on the other system and writing into that system's shared memory is accomplished by a C program running infinitely in the background.
</p>


<p>
<b>License:</b> OSMC-PL v1.2 2017<br /><br />

</p>

</html>"),
    uses(Modelica(version = "3.2.2")));
end Overview;