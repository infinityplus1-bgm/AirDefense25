#include <Arduino.h>

// Define pins for Motor 1 (Motor A)
#define dirPin1 32
#define stepPin1 33
#define enaPin1 25 // Enable pin for Motor 1

// Define pins for Motor 2 (Motor B)
#define dirPin2 17
#define stepPin2 16
#define enaPin2 4  // Enable pin for Motor 2



// laser setup
const int ledPin = 5;
// setting PWM properties
const int freq = 5000;
const int resolution = 8;



// Define the size of our expected message (Laser PWM + Motor A steps + Motor B steps)
const int MESSAGE_SIZE = 5;
byte receivedBytes[MESSAGE_SIZE];

/**
 * @brief Initializes serial communication and sets up motor driver pins.
 */
void setup() {
    // Initialize serial communication at 115200 baud rate, matching ROS 2 node
    Serial.begin(115200);
    while (!Serial); // Wait for serial port to connect (especially for native USB ports)
    Serial.println("ESP32 Serial Receiver Ready");

    // Set Motor 1 pins as outputs
    pinMode(stepPin1, OUTPUT);
    pinMode(dirPin1, OUTPUT);
    pinMode(enaPin1, OUTPUT);
    digitalWrite(enaPin1, LOW); // Enable Motor 1 driver (LOW typically enables A4988/DRV8825)

    // Set Motor 2 pins as outputs
    pinMode(stepPin2, OUTPUT);
    pinMode(dirPin2, OUTPUT);
    pinMode(enaPin2, OUTPUT);
    digitalWrite(enaPin2, LOW); // Enable Motor 2 driver

    ledcAttach(ledPin, freq, resolution);
    ledcWrite(ledPin, 0);
}

/**
 * @brief Rotates a specified motor by a given number of steps, handling direction.
 *
 * @param motor The ID of the motor to control (1 for Motor A, 2 for Motor B).
 * @param steps The number of steps to rotate. Positive values for one direction,
 * negative values for the opposite direction.
 */
void rotateSteps(int motor, int16_t steps) {
int stepPin;
int dirPin;

// Determine the correct step and direction pins based on motor ID
if (motor == 1) { // Motor A
    stepPin = stepPin1;
    dirPin = dirPin1;
} else if (motor == 2) { // Motor B
    stepPin = stepPin2;
    dirPin = dirPin2;
} else {
    Serial.println("Invalid motor ID provided to rotateSteps.");
    return; // Exit if motor ID is invalid
}

// Set the direction based on the sign of the steps
if (steps < 0) {
    digitalWrite(dirPin, HIGH); // Set direction for negative steps (e.g., counter-clockwise)
} else {
    digitalWrite(dirPin, LOW);  // Set direction for positive steps (e.g., clockwise)
}

// Use the absolute value of steps for the rotation loop
long absoluteSteps = abs(steps);

// Perform the steps
for (long i = 0; i < absoluteSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(400); // Adjust delay for desired speed (lower delay = faster)
    digitalWrite(stepPin, LOW);
    delayMicroseconds(400);
}
}

/**
 * @brief Rotates a specified motor by a given number of degrees.
 * This function calculates steps from degrees and then calls rotateSteps.
 *
 * @param motor The ID of the motor to control (1 for Motor A, 2 for Motor B).
 * @param degrees The number of degrees to rotate. Positive for one direction,
 * negative for the opposite.
 */
void rotateDegrees(int motor, float degrees) {
// Calculate steps from degrees. Assuming 40000 steps per 360 degrees (e.g., 1/128 microstepping with 200 step/rev motor)
// Adjust the (40000.0 / 360.0) constant based on your motor's steps per revolution and microstepping setting.
long steps = round(degrees * (40000.0 / 360.0));

// Call the rotateSteps function with the calculated steps
rotateSteps(motor, steps);
}


/**
 * @brief Main loop for reading serial data and controlling motors.
 */
void loop() {
// Check if enough bytes are available in the serial buffer for a complete message
if (Serial.available() >= MESSAGE_SIZE) {
    // Read the 5 bytes from the serial buffer
    Serial.readBytes(receivedBytes, MESSAGE_SIZE);

    // Byte 0: Laser PWM value (unsigned 8-bit)
    uint8_t laserPwm = receivedBytes[0];

    // Bytes 1-2: Motor A steps (signed 16-bit)
    // Reconstruct the signed 16-bit integer from two bytes.
    // Assuming little-endian format (LSB first, then MSB)
    int16_t motorASteps = (int16_t) (receivedBytes[1] | (receivedBytes[2] << 8));

    // Bytes 3-4: Motor B steps (signed 16-bit)
    int16_t motorBSteps = (int16_t) (receivedBytes[3] | (receivedBytes[4] << 8));

    // Print the received values to the Serial Monitor for debugging
    Serial.print("Received - ");
    Serial.print("Laser PWM: ");
    Serial.print(laserPwm);
    Serial.print(", Motor A Steps: ");
    Serial.print(motorASteps);
    Serial.print(", Motor B Steps: ");
    Serial.println(motorBSteps);

    // Control Motor A (Motor ID 1) based on received steps
    if (motorASteps != 0) { // Only move if steps are not zero
    rotateSteps(1, motorASteps);
    }

    // Control Motor B (Motor ID 2) based on received steps
    if (motorBSteps != 0) { // Only move if steps are not zero
    rotateSteps(2, motorBSteps);
    }


    if (laserPwm > 0){
            Serial.print("Turning laser ON with PWM: ");
            Serial.println(laserPwm);
            ledcWrite(ledPin, laserPwm); // Set laser intensity
            delay(1000);                       // Keep laser on for 1 second
            Serial.println("Turning laser OFF.");
            ledcWrite(ledPin, 0);        // Turn laser off
    }
    // For example: analogWrite(laserPin, laserPwm);
}
}
