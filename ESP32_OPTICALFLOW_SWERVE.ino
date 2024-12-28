#include "Optical_Flow_Sensor.h"
#include <HardwareSerial.h>

HardwareSerial SerialPort(2);

Optical_Flow_Sensor flow(10, PAA5100);

const double height_from_gnd = 20.0;    //Height in mm
const float scaler = 7.2;               //Adjust for sensitivity for different surfaces

// Sensor FOV is 42 degrees
// height_from_gnd*2*tan(42/2) is dist covered by sensor
// Sensor resolution is 35*35

const double scale_factor = height_from_gnd * 2.0 * tan(42.0 / 2.0) / (35.0 * scaler);

const float THRESHOLD = 200.0;
/* after 20-ish times of testing, value for y axis was off by 0.8cm
*/
//const float ALPHA = 0.8;
//const float BETA = 0.35;

const float ALPHA = 0.85;
const float BETA = 0.38;

float globalX = 0.0;
float globalY = 0.0;

float prevFilteredDX = 0.0;
float prevFilteredDY = 0.0;

float lowPassFilter(float prevValue, float newValue) {
  return ALPHA * newValue + BETA * prevValue;
}

TaskHandle_t Task1;

// Position and Orientation Variables
float X_global = 0.0;
float Y_global = 0.0;
float orientation = 0.0; // in degrees

unsigned long previousMillis = 0; // For timing
const float dt = 0.1; // Time step in seconds (adjust based on loop frequency)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, 16, 17);
  rgbLedWrite(38, 0, 0, 0);  // Red
  delay(10);
  rgbLedWrite(38, 60, 0, 0);  // Red
  delay(10);
  //Serial.println("Yeah");

  while (!flow.begin()) {
    //Serial.println("No optical flow found");
    delay(10);
  }

  rgbLedWrite(38, 0, 60, 0);  // Red
  delay(10);

  xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, &Task1, 1);
}

void loop() {
  // put your main code here, to run repeatedly:
  int16_t deltaX, deltaY;
  flow.readMotionCount(&deltaX, &deltaY);

  float localVelocityX = deltaX / 100.0; // Convert to cm per second
  float localVelocityY = deltaY / 100.0; // Convert to cm per second

  float filteredDeltaX = lowPassFilter(prevFilteredDX, deltaX);
  float filteredDeltaY = lowPassFilter(prevFilteredDY, deltaY);

  prevFilteredDX = filteredDeltaX;
  prevFilteredDY = filteredDeltaY;

  if (abs(filteredDeltaX - deltaX) < THRESHOLD && abs(filteredDeltaY - deltaY) < THRESHOLD){
    globalX += scale_factor * filteredDeltaX;
    globalY += scale_factor * filteredDeltaY;
    //    Serial.print("Global X: ");
    //    Serial.println(globalX/100);
    //    Serial.print("Global Y: ");
    //    Serial.println(globalY/100);
    //    Serial.print("\n");
  }
  else {
    //Serial.println("Spike in readings detected");
  }
  //rgbLedWrite(random(5,255), 0, random(5,255), 0);
  delay(2);
}

void Task1code(void* pvParameters) {
  for (;;) {
    SerialPort.write('X');
    SerialPort.print(globalX / 100);
    SerialPort.write('C');
    delay(2);
    SerialPort.write('Y');
    SerialPort.print(globalY / 100);
    SerialPort.write('D');
    delay(2);
  }
}
