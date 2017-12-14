#include <avr/interrupt.h> 
#include <avr/io.h> 

#define BAUD 115200                           // define baud
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)    // set baudrate value for UBRR
  
#ifndef F_CPU
#define F_CPU 16000000UL                    // set the CPU clock
#endif

int Output;
void setup()
{
   pinMode(13, OUTPUT); 

   UBRR0H = (BAUDRATE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register 
   UBRR0L = (BAUDRATE); // Load lower 8-bits of the baud rate value into the low byte of the UBRR register 
   UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01); // Use 8-bit character sizes 
   UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);   // Turn on the transmission, reception, and Receive interrupt      
   interrupts();
}

void loop()
{
  Output = Serial.read();
}

ISR(USART_RX_vect)
{  
  Serial.write(Output);
}

