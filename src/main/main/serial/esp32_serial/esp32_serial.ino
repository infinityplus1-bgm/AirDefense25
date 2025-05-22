// Define the size of our expected message
const int MESSAGE_SIZE = 5;
byte receivedBytes[MESSAGE_SIZE];

void setup() {
  Serial.begin(115200); // Initialize serial communication at the same baud rate as ROS 2 node
  while (!Serial); // Wait for serial port to connect. Needed for native USB port only
  Serial.println("ESP32 Serial Receiver Ready");
}

void loop() {
  if (Serial.available() >= MESSAGE_SIZE) {
    // Read the 5 bytes
    Serial.readBytes(receivedBytes, MESSAGE_SIZE);

    // Byte 0: Laser PWM (unsigned 8-bit)
    uint8_t laserPwm = receivedBytes[0];

    // Bytes 1-2: Motor A steps (signed 16-bit)
    // We need to reconstruct the signed 16-bit integer from two bytes.
    // Assuming little-endian format (LSB first, then MSB)
    int16_t motorASteps = (int16_t) (receivedBytes[1] | (receivedBytes[2] << 8));

    // Bytes 3-4: Motor B steps (signed 16-bit)
    int16_t motorBSteps = (int16_t) (receivedBytes[3] | (receivedBytes[4] << 8));

    Serial.print("Received - ");
    Serial.print("Laser PWM: ");
    Serial.print(laserPwm);
    Serial.print(", Motor A Steps: ");
    Serial.print(motorASteps);
    Serial.print(", Motor B Steps: ");
    Serial.println(motorBSteps);
  }
}