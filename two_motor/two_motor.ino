#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Motor control pins for NodeMCU
int motorPin1 = D5;  // IN1 (Forward, Left)
int motorPin2 = D6;  // IN2 (Backward, Right)

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize the MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Set accelerometer and gyroscope ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  
  // Set motor pins as output
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  Serial.println("Ready to control motor with accelerometer.");
  delay(100);
}

void loop() {
  // Get sensor events (accelerometer, gyroscope, temperature)
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Print accelerometer values for debugging
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print("\tY: ");
  Serial.println(a.acceleration.y);

  // Control motor direction based on accelerometer X and Y values
  if (a.acceleration.x > 3 && abs(a.acceleration.y) < 3) {
    // Move forward (X-axis positive, Y-axis near 0)
    digitalWrite(motorPin1, HIGH);  // Motor forward
    digitalWrite(motorPin2, LOW);   // Motor stop backward
    Serial.println("Motor moving forward");
  } else if (a.acceleration.x < -3 && abs(a.acceleration.y) < 3) {
    // Move backward (X-axis negative, Y-axis near 0)
    digitalWrite(motorPin1, LOW);   // Motor stop forward
    digitalWrite(motorPin2, HIGH);  // Motor backward
    Serial.println("Motor moving backward");
  } else if (a.acceleration.y > 3 && abs(a.acceleration.x) < 3) {
    // Move right (Y-axis positive, X-axis near 0)
    digitalWrite(motorPin1, HIGH);  // Motor right
    digitalWrite(motorPin2, LOW);   // Motor stop left
    Serial.println("Motor moving right");
  } else if (a.acceleration.y < -3 && abs(a.acceleration.x) < 3) {
    // Move left (Y-axis negative, X-axis near 0)
    digitalWrite(motorPin1, LOW);   // Motor stop right
    digitalWrite(motorPin2, HIGH);  // Motor left
    Serial.println("Motor moving left");
  } else {
    // Stop motor (if neither X nor Y exceeds threshold)
    digitalWrite(motorPin1, LOW);   // Motor stop forward
    digitalWrite(motorPin2, LOW);   // Motor stop backward
    Serial.println("Motor stopped");
  }

  delay(500);  // Wait before the next reading
}
