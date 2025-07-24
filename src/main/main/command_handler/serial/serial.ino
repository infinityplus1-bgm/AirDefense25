#include <Arduino.h>
#include <AccelStepper.h> // Include the AccelStepper library

// Define pins for Motor 1 (Motor A)
#define dirPin1 32
#define stepPin1 33
#define enaPin1 25 // Enable pin for Motor 1

// Define pins for Motor 2 (Motor B)
#define dirPin2 17
#define stepPin2 16
#define enaPin2 4  // Enable pin for Motor 2

// Create AccelStepper objects for each motor
// AccelStepper::DRIVER specifies that we are using a step/dir driver (like A4988, DRV8825)
AccelStepper stepperA(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper stepperB(AccelStepper::DRIVER, stepPin2, dirPin2);

// Laser setup
const int ledPin = 5;
// Setting PWM properties
const int freq = 5000;
const int resolution = 8;

// Define the size of our expected message (Laser PWM + Motor A steps + Motor B steps)
const int MESSAGE_SIZE = 5;
byte receivedBytes[MESSAGE_SIZE];

/**
 * @brief Initializes serial communication and sets up motor driver pins.
 * Configures AccelStepper objects with max speed and acceleration.
 */
void setup() {
    // Initialize serial communication at 115200 baud rate, matching ROS 2 node
    Serial.begin(115200);
    while (!Serial); // Wait for serial port to connect (especially for native USB ports)
    Serial.println("ESP32 Serial Receiver Ready");

    // Configure AccelStepper for Motor A
    // Set maximum speed in steps per second. Adjust this value based on your motor and driver capabilities.
    stepperA.setMaxSpeed(1000.0); // Example: 1000 steps/second
    // Set acceleration in steps per second per second. Adjust for smoother starts/stops.
    stepperA.setAcceleration(500.0); // Example: 500 steps/second^2

    // Configure AccelStepper for Motor B
    stepperB.setMaxSpeed(2000.0); // Example: 1000 steps/second
    stepperB.setAcceleration(1600.0); // Example: 500 steps/second^2

    // Set enable pins as outputs and enable drivers (LOW typically enables A4988/DRV8825)
    pinMode(enaPin1, OUTPUT);
    digitalWrite(enaPin1, LOW); // Enable Motor 1 driver
    pinMode(enaPin2, OUTPUT);
    digitalWrite(enaPin2, LOW); // Enable Motor 2 driver

    // Setup laser PWM
    ledcAttach(ledPin, freq, resolution); // Attach the LED pin to channel 0
    ledcWrite(ledPin, 0); // Ensure laser is off initially
}

/**
 * @brief Main loop for reading serial data and controlling motors and laser.
 * The motor movements are non-blocking due to AccelStepper's run() method.
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

        // Set target positions for motors.
        // AccelStepper's move() adds steps to the current position.
        // AccelStepper's moveTo() sets an absolute target position.
        // Here, we'll use move() for relative steps as in the original logic.
        if (motorASteps != 0) {
            stepperA.move(motorASteps); // Move Motor A by the specified number of steps
        }

        if (motorBSteps != 0) {
            stepperB.move(motorBSteps); // Move Motor B by the specified number of steps
        }

        // Control laser
        if (laserPwm > 0) {
            Serial.print("Turning laser ON with PWM: ");
            Serial.println(laserPwm);
            ledcWrite(ledPin, laserPwm); // Set laser intensity using channel 0
            // Note: The original code had a blocking delay(1000) here.
            // For truly non-blocking operation, you might want to manage laser
            // on/off with a timer or by sending separate commands for duration.
            // For now, it will turn on, then immediately turn off after processing the serial data.
            // If you need the laser to stay on for a duration, you'll need to implement
            // a state machine or a non-blocking timer.
            delay(1000);
            Serial.println("Turning laser OFF.");
            ledcWrite(ledPin, 0); // Turn laser off
        }
    }

    // IMPORTANT: Call run() for each stepper in the loop.
    // This function must be called as frequently as possible to ensure smooth motor operation.
    // It will make one step if necessary and return, allowing other code to execute.
    stepperA.run();
    stepperB.run();
}
