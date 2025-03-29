#include <Arduino.h>
#include <Servo.h>
#include "UARTProtocol.h"

Servo servo1;
Servo servo2;

const int pin1 = 2;
const int pin2 = 3;

const float alpha = 0.1;

// Initialize Protocol Object
UARTProtocol protocol(Serial, 0xaa, 10, 115200);

// Smooth Movement Formula: 𝚹(new) = ⍺ * 𝚹 + (1 - ⍺) * 𝚹(current)
void smoothExp(Servo &servo, byte targetAngle)
{
  float currentAngle = servo.read();

  while (abs(currentAngle - targetAngle) > 0.1)
  {
    currentAngle = alpha * targetAngle + (1 - alpha) * currentAngle;
    servo.write(int(currentAngle));
    delay(15);
  }

  servo.write(targetAngle);
}

void moveToAngle(Servo &servo, int ID, byte receivedAngle)
{
  Serial.print("Moving Servo ");
  Serial.print(ID);
  Serial.print(" from ");
  Serial.print(servo.read());
  Serial.print(" to ");
  Serial.println(receivedAngle);

  smoothExp(servo, receivedAngle);
}

void setup()
{
  // Set Servo Pins
  servo1.attach(pin1);
  servo2.attach(pin2);

  // Initialize it to 0 degrees
  servo1.write(0);
  servo2.write(0);

  protocol.begin();
  Serial.println("All Servos Set a 0 Degrees");
}

void loop()
{
  if (protocol.isAvailable())
  {
    uint8_t command;
    byte receivedAngle;

    if (protocol.readCommand(command))
    {
      if (protocol.readData(&receivedAngle, 1))
      {
        receivedAngle = constrain(receivedAngle, 0, 180);

        if (command == 0x01)
          moveToAngle(servo1, 1, receivedAngle);

        else if (command == 0x02)
          moveToAngle(servo2, 2, receivedAngle);

        else if (command == 0x03)
          Serial.println("Invalid Motor ID");
      }
    }
  }
}