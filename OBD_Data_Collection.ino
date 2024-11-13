#include <CAN.h>

// Set to true if using standard 11-bit addressing; false for extended 29-bit addressing.
const bool useStandardAddressing = true;

// Variables to store retrieved values
float engineRPM = -1;
float vehicleSpeed = -1;
float throttlePosition = -1;
float previousSpeed = 0;
float acceleration = 0;
unsigned long lastTimestamp = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("CAN OBD-II Data Retrieval");

  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  // Set CAN filter based on addressing
  if (useStandardAddressing) {
    CAN.filter(0x7e8);
  } else {
    CAN.filterExtended(0x18daf110);
  }
}

void loop() {
  unsigned long currentTimestamp = millis();
  
  // Request RPM
  engineRPM = requestPID(0x0C);
  delay(50);

  // Request Vehicle Speed
  vehicleSpeed = requestPID(0x0D);
  delay(50);

  // Request Throttle Position
  throttlePosition = requestPID(0x11);
  delay(50);

  // Calculate acceleration (change in speed over time)
  if (lastTimestamp > 0) {
    float deltaTime = (currentTimestamp - lastTimestamp) / 1000.0; // seconds
    acceleration = (vehicleSpeed - previousSpeed) / deltaTime;
  }

  // Update previous speed and timestamp for the next calculation
  previousSpeed = vehicleSpeed;
  lastTimestamp = currentTimestamp;

  // Print out the retrieved values in the required format
  Serial.print(engineRPM);
  Serial.print(",");
  Serial.print(vehicleSpeed);
  Serial.print(",");
  Serial.print(throttlePosition);
  Serial.print(",");
  Serial.print(acceleration);
  Serial.println();

  delay(1000); // Delay to repeat every 1 second
}

// Function to request specific PID and parse its response
float requestPID(byte pid) {
  // Begin CAN packet based on addressing
  if (useStandardAddressing) {
    CAN.beginPacket(0x7df, 8);
  } else {
    CAN.beginExtendedPacket(0x18db33f1, 8);
  }

  CAN.write(0x02); // Number of additional bytes
  CAN.write(0x01); // Show current data
  CAN.write(pid);  // Requested PID
  CAN.endPacket();

  // Wait for response and validate it
  unsigned long start = millis();
  while (millis() - start < 100) {  // 100 ms timeout
    if (CAN.parsePacket()) {
      if (CAN.read() >= 3 && CAN.read() == 0x41 && CAN.read() == pid) {
        if (pid == 0x0C) { // Engine RPM
          return ((CAN.read() * 256) + CAN.read()) / 4.0;
        } else if (pid == 0x0D) { // Vehicle Speed
          return CAN.read();
        } else if (pid == 0x11) { // Throttle Position
          return (CAN.read() * 100.0) / 255.0;
        }
      }
    }
  }

  // Return -1 if no valid data is received
  return -1;
}
