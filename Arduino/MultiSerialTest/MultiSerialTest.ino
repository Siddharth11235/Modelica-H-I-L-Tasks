// Include AsciiMassagePacker.h for ASCII format massage packing.
#include <AsciiMassagePacker.h>

// Instantiate an AsciiMassagePacker for packing massages.
AsciiMassagePacker outbound;

void setup() {

  // Start the Serial protocol at 57600 baud.
  Serial.begin(9600);

}

void loop() {

  // This is just an example of sending dummy values.
  // You must customize with your own addresses and values.
  // The application receiving the massage must expect a massage 
  // with the same address ("value" in this case).
  outbound.addInt( analogRead(A5)*2 ); // Add a reading of analog 0.
  outbound.endPacket(); // End the packet.

  // Send the packet with the Serial protocol.
  Serial.write( outbound.buffer(), outbound.size() );

  // You can view the sent massages in Arduino's Serial Monitor
  // because Arduino's Serial Monitor uses the ASCII format.

  // A little delay to slow down everything for this example.
  // It is not required in your use.
  delay(50);

}

