#include <Wire.h>

#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

#include <DFRobot_LIS2DH12.h>

DFRobot_LIS2DH12 LIS; //Accelerometer

int buzzerPin = 4;

float steadyStateMax = 300;
float steadyStateMin = -500;

int stepCount = 0;


// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
#define RST_PIN -1

SSD1306AsciiWire oled;

void setup(){
  Wire.begin();
  Serial.begin(115200);
  while(!Serial);
  delay(100);

  // Set measurement range
  // Ga: LIS2DH12_RANGE_2GA
  // Ga: LIS2DH12_RANGE_4GAs
  // Ga: LIS2DH12_RANGE_8GA
  // Ga: LIS2DH12_RANGE_16GA
  while(LIS.init(LIS2DH12_RANGE_16GA) == -1){  //Equipment connection exception or I2C address error
    Serial.println("No I2C devices found");
    delay(1000);
  }

  setupDisplay();

  pinMode(buzzerPin, OUTPUT);
}

void loop(){
  acceleration();

  tone(buzzerPin, 1000);
  delay(1000);
  noTone(buzzerPin);  
}

void setupDisplay() 
{
  Wire.setClock(400000L);

#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0

  oled.setFont(Adafruit5x7);

  uint32_t m = micros();
  oled.clear();
  oled.println("Health Hackers");
}

/*!
 *  @brief Print the position result.
 */
void acceleration(void)
{
  int16_t x, y, z;


  delay(100);
  LIS.readXYZ(x, y, z);
  LIS.mgScale(x, y, z);
  // Serial.print(x);
  // Serial.print(",");
  // Serial.print(y);
  // Serial.print(",");
  // Serial.print(z);
  // Serial.println(",");
  // Serial.println(y);

  if (y > steadyStateMax || y < steadyStateMin) // y is outside of steady state range
  {
    //Serial.println("ABOVE steady state");
    delay(100);
    if (y <= steadyStateMax || y > steadyStateMin) // y falls back into the steady state range
    {
      stepCount += 1;
      //Serial.println("UNDER steady state");
      Serial.println(stepCount);
      delay(100);
    }
  }
}