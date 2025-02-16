#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define BUZZER_PIN 12 // Use GPIO 12 for ESP8266

Adafruit_MPU6050 mpu1, mpu2;

unsigned long prevTime = 0;
float alpha = 0.98; // Complementary filter strength

float roll1 = 0, pitch1 = 0, yaw1 = 0;
float roll2 = 0, pitch2 = 0, yaw2 = 0;

void setup()
{
  Serial.begin(115200);
  Wire.begin(4, 5); // SDA = GPIO 4, SCL = GPIO 5 for ESP8266

  Serial.println("Initializing MPU6050 sensors...");

  if (!mpu1.begin(0x68))
  {
    Serial.println("Failed to find MPU6050 #1 (0x68)");
    while (1)
      ;
  }

  if (!mpu2.begin(0x69))
  {
    Serial.println("Failed to find MPU6050 #2 (0x69)");
    while (1)
      ;
  }

  pinMode(BUZZER_PIN, OUTPUT);
  Serial.println("MPU6050s initialized!");
}

void loop()
{
  calculate_orientation(mpu1, roll1, pitch1, yaw1);
  delay(5);
  calculate_orientation(mpu2, roll2, pitch2, yaw2);
  delay(5);

  if (calculate_pitch_difference(pitch1, pitch2))
  {
    tone(BUZZER_PIN, 1800);
    delay(10);
  }
  else
  {
    noTone(BUZZER_PIN);
  }

  delay(5);
}

bool calculate_pitch_difference(float pitch1, float pitch2)
{
  return (abs(pitch1 - pitch2) > 30);
}

void calculate_orientation(Adafruit_MPU6050 &mpu, float &roll, float &pitch, float &yaw)
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currTime = millis();
  float dt = (currTime - prevTime) / 1000.0;
  prevTime = currTime;

  float rollAcc = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float pitchAcc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  roll = alpha * (roll + g.gyro.x * dt) + (1 - alpha) * rollAcc;
  pitch = alpha * (pitch + g.gyro.y * dt) + (1 - alpha) * pitchAcc;
  yaw += g.gyro.z * dt;
}
