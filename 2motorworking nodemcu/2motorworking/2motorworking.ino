#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Motor control pins for NodeMCU
int motor1Pin1 = D5;  // Motor 1 Forward
int motor1Pin2 = D6;  // Motor 1 Backward
int motor2Pin1 = D7;  // Motor 2 Forward
int motor2Pin2 = D8;  // Motor 2 Backward

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
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  Serial.println("Ready to control motors with accelerometer.");
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

  // Left/Right control based on Y-axis (this logic always takes precedence)
  if (a.acceleration.y > 3) {
    // Rotate right motor only to move right when Y-axis is positive
    digitalWrite(motor1Pin1, LOW);  // Motor 1 stop forward
    digitalWrite(motor1Pin2, LOW);  // Motor 1 stop backward
    digitalWrite(motor2Pin1, HIGH); // Motor 2 forward (right)
    digitalWrite(motor2Pin2, LOW);  // Motor 2 stop backward
    Serial.println("Motor 2 moving right");
  } else if (a.acceleration.y < -3) {
    // Rotate left motor only to move left when Y-axis is negative
    digitalWrite(motor1Pin1, HIGH); // Motor 1 forward (left)
    digitalWrite(motor1Pin2, LOW);  // Motor 1 stop backward
    digitalWrite(motor2Pin1, LOW);  // Motor 2 stop forward
    digitalWrite(motor2Pin2, LOW);  // Motor 2 stop backward
    Serial.println("Motor 1 moving left");
  } else {
    // Stop motors for left/right control if Y-axis is near 0
    digitalWrite(motor1Pin1, LOW);  // Motor 1 stop forward
    digitalWrite(motor1Pin2, LOW);  // Motor 1 stop backward
    digitalWrite(motor2Pin1, LOW);  // Motor 2 stop forward
    digitalWrite(motor2Pin2, LOW);  // Motor 2 stop backward
    Serial.println("Both motors stopped (Y-axis)");
  }

  // Forward/Backward control based on X-axis (only applies if Y-axis isn't moving significantly)
  if (a.acceleration.x > 3 && abs(a.acceleration.y) < 3) {
    // Move both motors forward when X-axis is positive and Y-axis is neutral
    digitalWrite(motor1Pin1, HIGH);  // Motor 1 forward
    digitalWrite(motor1Pin2, LOW);   // Motor 1 stop backward
    digitalWrite(motor2Pin1, HIGH);  // Motor 2 forward
    digitalWrite(motor2Pin2, LOW);   // Motor 2 stop backward
    Serial.println("Both motors moving forward");
  } else if (a.acceleration.x < -3 && abs(a.acceleration.y) < 3) {
    // Move both motors backward when X-axis is negative and Y-axis is neutral
    digitalWrite(motor1Pin1, LOW);   // Motor 1 stop forward
    digitalWrite(motor1Pin2, HIGH);  // Motor 1 backward
    digitalWrite(motor2Pin1, LOW);   // Motor 2 stop forward
    digitalWrite(motor2Pin2, HIGH);  // Motor 2 backward
    Serial.println("Both motors moving backward");
  } else if (abs(a.acceleration.x) < 3 && abs(a.acceleration.y) < 3) {
    // Stop both motors if X-axis is near 0 and Y-axis is also near 0
    digitalWrite(motor1Pin1, LOW);   // Motor 1 stop forward
    digitalWrite(motor1Pin2, LOW);   // Motor 1 stop backward
    digitalWrite(motor2Pin1, LOW);   // Motor 2 stop forward
    digitalWrite(motor2Pin2, LOW);   // Motor 2 stop backward
    Serial.println("Both motors stopped (X-axis and Y-axis)");
  }

  delay(100);  // Wait before the next reading
}
