#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_HMC5883_U.h>

// Define the GPS and SD card objects
SoftwareSerial mySerial(3, 2); // RX, TX
Adafruit_GPS GPS(&mySerial);
File gpsLogFile;
File waypointsFile;

Servo escL;                       // Servo object to control ESC 1
Servo escR;                       // Servo object to control ESC 2

// ESC parameters
const int escREVPulse = 1000;    // Minimum pulse width for ESC (microseconds)
const int escFORPulse = 2000;    // Maximum pulse width for ESC (microseconds)
const int escNeutralPulse = 1500; // Neutral pulse width for ESC (microseconds)


// Define compass object
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);

// Define ESC pins
const int leftESC = 2;  // Connect to left motor ESC
const int rightESC = 3; // Connect to right motor ESC

// Define motor speeds
const int forwardSpeed = 2000; // microseconds
const int turningSpeed = 100; // Adjust as needed

// Define waypoint coordinates
float waypoints[3][2] = {
  {-33.0286, 18.7915}, // Waypoint 1
  {-33.0259, 18.79209},   // Waypoint 2
  {-33.0297, 18.790241}    // Waypoint 3
};

int currentWaypoint = 0;

float xOffset = -7.0; // Replace with your X offset
float yOffset = -26.0; // Replace with your Y offset
float zOffset = 21.0; // Replace with your Z offset

void setup() {
  Serial.begin(9600);
  escL.attach(leftESC);    // Attach servo object for ESC 1
  escR.attach(rightESC);    // Attach servo object for ESC 2
  GPS.begin(9600);

  escL.writeMicroseconds(escNeutralPulse); // Send neutral signal to initialize ESC L
  escR.writeMicroseconds(escNeutralPulse); // Send neutral signal to initialize ESC R
  
  delay(2000);                    // Wait for ESCs to initialize

  if (!SD.begin(10)) {
    Serial.println("SD card initialization failed!");
    return;
  }

  waypointsFile = SD.open("waypoints.csv");
  if (!waypointsFile) {
    Serial.println("Failed to open waypoints file");
    return;
  }

  gpsLogFile = SD.open("gps_log.txt", FILE_WRITE);
  if (!gpsLogFile) {
    Serial.println("Failed to open GPS log file");
    return;
  }

  // Initialize ESCs
  
 // pinMode(leftESC, OUTPUT);
 // pinMode(rightESC, OUTPUT);
  stopMotors();

  // Initialize compass
  if (!compass.begin()) {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    while (1);
  }
}

void loop() {
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
    // Log GPS data to the SD card
    logGPSData();
  }

  // Check if we have reached the current waypoint
  float currentLat = GPS.latitudeDegrees;
  float currentLon = GPS.longitudeDegrees;
  float targetLat = waypoints[currentWaypoint][0];
  float targetLon = waypoints[currentWaypoint][1];

  if (distance(currentLat, currentLon, targetLat, targetLon) < 0.01) {
    currentWaypoint++;
    if (currentWaypoint >= 3) {
      Serial.println("Reached all waypoints.");
      stopMotors();
      gpsLogFile.close();
      while (1); // End the loop
    }
  }

  // Calculate desired heading using compass
  sensors_event_t event;
  compass.getEvent(&event);

    float magX = event.magnetic.x - xOffset;
  float magY = event.magnetic.y - yOffset;
  float magZ = event.magnetic.z - zOffset;
  
  float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
  float desiredHeading = calculateHeading(currentLat, currentLon, targetLat, targetLon);
  float headingError = desiredHeading - heading;

  // Adjust motor speeds based on heading error
  int leftSpeed = forwardSpeed;
  int rightSpeed = forwardSpeed;

  if (abs(headingError) > 30) {
    if (headingError < 0) {
      // Turn left
      leftSpeed = escREVPulse;
      rightSpeed = escFORPulse;
    } else {
      // Turn right
      leftSpeed = escFORPulse;
      rightSpeed = escREVPulse;
    }
  }

    if (abs(headingError) > 10) {
    if (headingError < 0) {
      // Turn left
      leftSpeed = escNeutralPulse;
      rightSpeed = escFORPulse;
    } else {
      // Turn right
      leftSpeed = escFORPulse;
      rightSpeed = escNeutralPulse;
    }
  }

  // Control ESCs
  escL.writeMicroseconds(leftSpeed);
  escR.writeMicroseconds(rightSpeed)

  // Print debug information
  Serial.print("Lat: ");
  Serial.print(currentLat, 6);
  Serial.print(" Lon: ");
  Serial.print(currentLon, 6);
  Serial.print(" Heading: ");
  Serial.print(heading);
  Serial.print(" Desired Heading: ");
  Serial.print(desiredHeading);
  Serial.print(" Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" Right Speed: ");
  Serial.println(rightSpeed);
}

void logGPSData() {
  gpsLogFile.print("Lat: ");
  gpsLogFile.print(GPS.latitudeDegrees, 6);
  gpsLogFile.print(" Lon: ");
  gpsLogFile.print(GPS.longitudeDegrees, 6);
  gpsLogFile.print(" Alt: ");
  gpsLogFile.print(GPS.altitude, 1);
  gpsLogFile.print(" Satellites: ");
  gpsLogFile.print(GPS.satellites);
  gpsLogFile.print(" Speed: ");
  gpsLogFile.print(GPS.speed);
  gpsLogFile.print(" Course: ");
  gpsLogFile.print(GPS.angle);
  gpsLogFile.println();
}

float calculateHeading(float lat1, float lon1, float lat2, float lon2) {
  float dLon = lon2 - lon1;
  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  float brng = atan2(y, x);
  brng = (brng * 180 / PI + 360);
  return fmod(brng, 360);
}

float distance(float lat1, float lon1, float lat2, float lon2) {
  float R = 6371; // Earth radius in km
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat / 2) * sin(dLat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float distance = R * c;
  return distance;
}

void stopMotors() {
  escL.writeMicroseconds(escNeutralPulse); // Send neutral signal to initialize ESC L
  escR.writeMicroseconds(escNeutralPulse); // Send neutral signal to initialize ESC R
}
