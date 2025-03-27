#include <Arduino.h>
#include <Servo.h>
#include "UARTProtocol.h"

Servo servo1;
Servo servo2;
Servo servo3;

const int pin1 = 2;
const int pin2 = 3;
const int pin3 = 4;

UARTProtocol protocol(Serial, 0xaa, 10, 115200);

void moveToAngle(Servo servo, int ID, byte receivedAngle)
{
  Serial.print("Moving Servo ");
  Serial.print(ID);
  Serial.print(" from ");
  Serial.print(servo.read());
  Serial.print(" to ");
  Serial.println(receivedAngle);

  servo.write(receivedAngle);
}

void setup()
{
  protocol.begin();

  servo1.attach(pin1);
  servo2.attach(pin2);
  servo3.attach(pin3);

  // Initialize it to 0 degrees
  servo1.write(0);
  servo2.write(0);
  servo3.write(0);

  Serial.println("All Servos Set a 0 Degrees");
}

void loop()
{
  if (protocol.IsAvailable())
  {
    uint8_t command;
    byte receivedAngle;

    if (protocol.ReadCommand(command))
    {
      if (protocol.ReadData(&receivedAngle, 1))
      {
        if (command == 0x01)
          moveToAngle(servo1, 1, receivedAngle);

        else if (command == 0x02)
          moveToAngle(servo2, 2, receivedAngle);

        else if (command == 0x03)
          moveToAngle(servo3, 3, receivedAngle);
      }
    }
  }
}