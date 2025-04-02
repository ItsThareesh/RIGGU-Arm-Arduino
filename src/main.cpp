#include <Arduino.h>
#include <Servo.h>
#include "UARTProtocol.h"
#include <TimerOne.h> // Library for timer interrupts on Arduino Mega

Servo servo1;
Servo servo2;

const int pin1 = 2;
const int pin2 = 3;

const float alpha = 0.1;

// Store the target and current angles for each servo
int targetAngle1 = 0;
int targetAngle2 = 0;
float currentAngle1 = 0;
float currentAngle2 = 0;

bool newCommandReceived = false;

// Initialize Protocol Object
UARTProtocol protocol(Serial, 0xaa, 10, 115200);

// Smooth Movement Formula: 𝚹(new) = ⍺ * 𝚹 + (1 - ⍺) * 𝚹(current)
void smoothExp(float &currentAngle, Servo &servo, int targetAngle)
{
  if (abs(currentAngle - targetAngle) > 0.1)
  {
    currentAngle = alpha * targetAngle + (1 - alpha) * currentAngle;
    servo.write(int(currentAngle));
  }
  else
  {
    servo.write(targetAngle);
    currentAngle = targetAngle;
  }
}

void manageServos()
{
  if (newCommandReceived)
  {
    bool allAnglesSet = true;
    
    if (currentAngle1 != targetAngle1)
    {
      smoothExp(currentAngle1, servo1, targetAngle1);
      allAnglesSet = false;
    }

    if (currentAngle2 != targetAngle2)
    {
      smoothExp(currentAngle2, servo2, targetAngle2);
      allAnglesSet = false;
    }

    if (allAnglesSet)
    {
      newCommandReceived = false;
      Timer1.detachInterrupt(); // Disable the timer interrupt when no new command is received
    }
  }
}

void setup()
{
  // Set Servo Pins
  servo1.attach(pin1);
  servo2.attach(pin2);

  // Initialize current and target angles using servo.read()
  currentAngle1 = targetAngle1 = servo1.read();
  currentAngle2 = targetAngle2 = servo2.read();

  protocol.begin();
  Serial.begin(115200);
  Serial.println("All Servos Set at Initial Positions");
}

void loop()
{
  if (protocol.isAvailable())
  {
    uint8_t command;
    int receivedAngle;

    if (protocol.readCommand(command))
    {
      if (protocol.readData((byte*)&receivedAngle, sizeof(int)))
      {
        receivedAngle = constrain(receivedAngle, 0, 180);

        if (command == 0x01)
          targetAngle1 = receivedAngle;
        else if (command == 0x02)
          targetAngle2 = receivedAngle;
        else if (command == 0x03)
          Serial.println("Invalid Motor ID");
        
        newCommandReceived = true;
        Timer1.attachInterrupt(manageServos); // Enable the timer interrupt when a new command is received
        Timer1.initialize(15000); // Ensure the timer is set to trigger every 15ms
      }
    }
  }
}
