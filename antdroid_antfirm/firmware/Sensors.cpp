#include <Wire.h>
#include "Configuration.h"  // Include the configuration file
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;
ros::Publisher ultrasonic_pub("ultrasonic_distance", new std_msgs::Int32());
ros::Publisher humidity_pub("humidity", new std_msgs::Float32());
ros::Publisher compass_pub("compass_data", new std_msgs::Float32[3]);
ros::Publisher soil_moisture_pub("soil_moisture", new std_msgs::Int32());


// Variables to store sensor data
int soilMoistureValue = 0;
float humidityValue = 0.0;
float compassX = 0.0, compassY = 0.0, compassZ = 0.0;
long ultrasonicDistance = 0;

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Initialize I2C communication
  Wire.begin();

  // Initialize sensors if needed (some sensors might need specific initialization)
  // Example: If a sensor requires specific commands, send them here
}

void loop() {
  // Check if there is data available to read from the Serial port
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // Read the command sent from terminal
    
    // If the command is "READ_SENSOR", read the sensor values
    if (command == "READ_SENSOR") {
      // Read the ultrasonic sensor data
      ultrasonicDistance = readUltrasonicSensor();
      
      // Read the humidity sensor data
      humidityValue = readHumiditySensor();
      
      // Read the compass sensor data
      readCompassSensor();
      
      // Read the soil moisture data
      soilMoistureValue = analogRead(SoilMoisturePin);

      // Convert soil moisture to percentage (assuming 0-1023 range)
      float soilMoisturePercent = map(soilMoistureValue, 0, 1023, 0, 100);

      // Print the sensor data to the Serial Monitor
      Serial.print("Ultrasonic Distance: ");
      Serial.print(ultrasonicDistance);
      Serial.println(" cm");

      Serial.print("Humidity: ");
      Serial.print(humidityValue);
      Serial.println(" %");

      Serial.print("Compass X: ");
      Serial.print(compassX);
      Serial.print(" Y: ");
      Serial.print(compassY);
      Serial.print(" Z: ");
      Serial.println(compassZ);

      Serial.print("Soil Moisture: ");
      Serial.print(soilMoisturePercent);
      Serial.println(" %");
    }
  }
}

long readUltrasonicSensor() {
  // Read data from the ultrasonic sensor (example)
  // Modify according to the ultrasonic sensor's protocol or library used
  Wire.beginTransmission(UltrasonicSensorAddress);
  Wire.write(0x00);  // Command to initiate reading (change if required)
  Wire.endTransmission();
  Wire.requestFrom(UltrasonicSensorAddress, 2);  // Assuming 2-byte data for distance
  if (Wire.available() >= 2) {
    long distance = Wire.read() << 8 | Wire.read();
    return distance;
  }
  return -1;  // Return error if no data available
}

float readHumiditySensor() {
  // Read data from the humidity sensor (example)
  // Modify according to the sensor's protocol or library used
  Wire.beginTransmission(HumiditySensorAddress);
  Wire.write(0x01);  // Command to request humidity data (change if required)
  Wire.endTransmission();
  Wire.requestFrom(HumiditySensorAddress, 2);  // Assuming 2-byte data for humidity
  if (Wire.available() >= 2) {
    int rawHumidity = Wire.read() << 8 | Wire.read();
    return rawHumidity * 0.1;  // Example conversion to percentage
  }
  return -1.0;  // Return error if no data available
}

void readCompassSensor() {
  // Read the compass data (example)
  // Modify according to the compass sensor's protocol or library used
  Wire.beginTransmission(CompassSensorAddress);
  Wire.write(0x03);  // Command to initiate reading (change if required)
  Wire.endTransmission();
  Wire.requestFrom(CompassSensorAddress, 6);  // Assuming 6 bytes for X, Y, Z axis data
  if (Wire.available() >= 6) {
    compassX = Wire.read() << 8 | Wire.read();
    compassY = Wire.read() << 8 | Wire.read();
    compassZ = Wire.read() << 8 | Wire.read();
  }
}
