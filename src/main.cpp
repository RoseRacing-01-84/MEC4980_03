#include <Arduino.h>
#include <stdint.h>
#include <SCMD.h>
#include <SCMD_config.h>
#include <Wire.h>
#include <PID_v1.h>
#include <SparkFun_Qwiic_OLED.h>


float lightval = 0.0;
float der = 0.0;
float rpm = 0.0;
bool prevPass = false;
int count = 0;

char pout[30];

SCMD myMotorDriver;

QwiicMicroOLED myOLED;

double Setpoint, Input, Output;
double Kp=0.04, Ki=0.10, Kd=0.02;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


bool lightpass() {
  lightval = analogRead(A0);
  if (lightval > 1700) {
    return true;
  } else {
    return false;
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  pinMode(A0, INPUT);
  pinMode(A5, OUTPUT);
  pinMode(6, INPUT_PULLDOWN);
  pinMode(5, INPUT_PULLDOWN);

  Input = 0.0;
  Setpoint = 100;
  myPID.SetMode(AUTOMATIC);

  myMotorDriver.settings.commInterface = I2C_MODE;
  myMotorDriver.settings.I2CAddress = 0x5D;
  while ( myMotorDriver.begin() != 0xA9 ) //Wait until a valid ID word is returned
  {
    Serial.println( "ID mismatch, trying again" );
    delay(500);
  }
  Serial.println( "ID matches 0xA9" );
  Serial.print("Waiting for enumeration...");
  while ( myMotorDriver.ready() == false );
  Serial.println("Done.");
  Serial.println();
  while ( myMotorDriver.busy() );
  myMotorDriver.enable();

  Serial.println();
   while(!myOLED.begin()){
    delay(1000);
  }
  Serial.println("Everything started!!!!!");
}

void loop() {
  if (digitalRead(5)) {
    Setpoint = Setpoint - 50;
    if (Setpoint < 101);
    Setpoint = 100;
  }
  if (digitalRead(6)) {
    Setpoint = Setpoint + 50;
    if (Setpoint > 399) {
      Setpoint = 400;
    }
  }

  unsigned long start = millis();
  while (millis() - start < 100) {
  if (lightpass() && !prevPass) {
    count++;
    prevPass = true; 
  }
  if (!lightpass() && prevPass) {
    prevPass = false;
    count++;
  }
}
  rpm = (count / 10.0) / ((millis() - start) / 60000.0);
  Serial.print(rpm);
  Serial.print(" RPM, Output: ");

  Input = rpm;
  myPID.Compute();
  Serial.println(Output);
  myMotorDriver.setDrive(0, 1, Output);

  myOLED.erase();
  sprintf(pout, "rpm: %.1f", rpm);
  myOLED.text(0,0,pout);
  sprintf(pout, "set: %.1f", Setpoint);
  myOLED.text(0,10,pout);
  myOLED.display();

  count = 0;
}
