#include <Arduino.h>
#include <Servo.h>
#include <TimerOne.h>
#include "UARTProtocol.h"

Servo servos[3];

int pins[3] = {2, 3, 4};

int initial_angles[3] = {180, 0, 140};
float targetAngles[3] = {0, 0, 0};
float currentAngles[3] = {0, 0, 0};
bool moving[3] = {false, false, false};

const float alpha = 0.1;

// Initialize Protocol Object
UARTProtocol protocol(Serial, 0xaa, 10, 115200);

void updateMotor()
{
  for (int i = 0; i < 3; i++)
  {
    if (abs(currentAngles[i] - targetAngles[i]) > 0.1)
    {
      // Smooth Movement Formula: 𝚹(new) = ⍺ * 𝚹(target) + (1 - ⍺) * 𝚹(current)
      currentAngles[i] = alpha * targetAngles[i] + (1 - alpha) * currentAngles[i];
      moving[i] = true;
      servos[i].write(int(currentAngles[i]));
    }
    else
    {
      currentAngles[i] = targetAngles[i];
      moving[i] = false;
    }
  }

  if (!moving[0] && !moving[1] && !moving[2])
    Timer1.detachInterrupt();
}

void logMovement()
{
  for (int i = 0; i < 3; i++)
  {
    Serial.print("Moving Servo ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(currentAngles[i], 0);
    Serial.print(" to ");
    Serial.println(targetAngles[i], 0);
  }

  Serial.println();
}

void moveMotorsAsync(byte angle1, byte angle2, byte angle3)
{
  targetAngles[0] = angle1;
  targetAngles[1] = angle2;
  targetAngles[2] = angle3;

  logMovement();

  Timer1.attachInterrupt(updateMotor);
}

void setup()
{
  for (int i = 0; i < 3; i++)
  {
    servos[i].attach(pins[i]);
    servos[i].write(initial_angles[i]); // Initialize it to default angles
  }

  Serial.println("Resetted to Initial Angles");

  protocol.begin();

  Timer1.initialize(15000); // 15 millisecond
}

void loop()
{
  if (protocol.isAvailable())
  {
    uint8_t command;
    byte receivedAngles[3];

    if (protocol.readCommand(command))
    {
      if (protocol.readData(receivedAngles, 3))
      {
        receivedAngles[0] = constrain(receivedAngles[0], 0, initial_angles[0]);
        receivedAngles[1] = constrain(receivedAngles[1], 0, 180);
        receivedAngles[2] = constrain(receivedAngles[2], 0, initial_angles[2]);

        moveMotorsAsync(initial_angles[0] - receivedAngles[0], receivedAngles[1], initial_angles[2] - receivedAngles[2]);
      }
    }
  }
}