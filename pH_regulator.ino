#include <Servo.h>

float feedInterval = 1500;   // seconds during feeding time
const byte servoPin1 = 9;      // pin used to command the servo motor 1
const byte servoPin2 = 10;     // pin used to command the servo motor 2
float pHUpperThreshold = 6.5;  
float pHLowerThreshold = 5.5;   
float pHvalue = analogRead(A0); // change this to the analog pin used 
unsigned long lastUpdateTime = 0;  //Track the last update time 
const unsigned long updateInterval = 1200000;    //Update once every 20 minutes

Servo servo1;
Servo servo2;

//stop the contents from flowing
void feederClose1() {
  servo1.write(180);
  delay(175);
  servo1.write(90);
}

//release the contents
void feederOpen1() {
  servo1.write(0);
  delay(175);
  servo1.write(90);
}

//stop the contents from flowing
void feederClose2() {
  servo2.write(180);
  delay(175);
  servo2.write(90);
}

//release the contents
void feederOpen2() {
  servo2.write(0);
  delay(175);
  servo2.write(90);
}

void setup() {
  Serial.begin(9600);
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  Serial.println("System initialized");
}

void loop() {
  if (millis() -  lastUpdateTime >= updateInterval); {
    Serial.println("Updating pH value...");
    Serial.println(pHvalue);
    if (pHvalue > pHUpperThreshold); {
      Serial.println("Releasing now");
      feederOpen1();
      delay(150);
      Serial.println("Closing now");
      feederClose1();
    }
    if (pHvalue < pHLowerThreshold); {
      Serial.println("Releasing now");
      feederOpen2();
      delay(150);
      Serial.println("Closing now");
      feederClose2();  
    }
  
  }
}
