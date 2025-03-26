#include "Configuration.h"
#include "Sensors.h"  // Include the Sensors header file
#include <std_msgs/String.h>
#ifdef ControlRos
#include "controlRos.h"

ros::Subscriber<antdroid_msgs::Walk> walk("/antfirm/walk", &ControlWalk);
ros::Subscriber<antdroid_msgs::Balance> balance("/antfirm/balance", &ControlBalance);
ros::Subscriber<antdroid_msgs::Rotate> rotate("/antfirm/rotate", &ControlRotate);
ros::Subscriber<antdroid_msgs::Speed> speed ("/antfirm/speed", &ControlChangeSpeed);
ros::Subscriber<antdroid_msgs::Height> height("/antfirm/height", &ControlChangeHeight);
ros::Subscriber<antdroid_msgs::Foot> footDistance("/antfirm/foot", &ControlChangeFootDistance);
ros::Subscriber<antdroid_msgs::Log> logLevel("/antfirm/log", &ControlChangeLogLevel);
ros::Subscriber<antdroid_msgs::Calibrate> calibration("/antfirm/calibrate", &ControlChangeCalibration);
ros::Subscriber<antdroid_msgs::Gait> gait("/antfirm/gait", &ControlChangeGait);
ros::Subscriber<antdroid_msgs::MoveLeg> moveLeg("/antfirm/move_leg", &ControlMoveLeg);
ros::Subscriber<std_msgs::Bool> attack("/antfirm/attack", &ControlAttack);
ros::Subscriber<std_msgs::Bool> sayHello("/antfirm/say_hello", &ControlSayHello);

// Subscriber for "READ_SENSOR" command
ros::Subscriber<std_msgs::String> read_sensor_subscriber("read_sensor_command", &ControlReadSensorCommand);

std_msgs::Bool is_new_message;
ros::Publisher pub_is_new_message("/antfirm/new_message", &is_new_message);

Control::Control(Hexapod* Antdroid)
{
    is_new_message.data = true;
}

void Control::Start(void)
{
    arduino.initNode();

    arduino.subscribe(walk);
    arduino.subscribe(balance);
    arduino.subscribe(rotate);
    arduino.subscribe(speed);
    arduino.subscribe(height);
    arduino.subscribe(footDistance);
    arduino.subscribe(logLevel);
    arduino.subscribe(calibration);
    arduino.subscribe(gait);
    arduino.subscribe(moveLeg);
    arduino.subscribe(attack);
    arduino.subscribe(sayHello);

    // Subscribe to the "read_sensor_command" topic
    arduino.subscribe(read_sensor_subscriber);

    arduino.advertise(pub_is_new_message);

    level_log = 0;
}

void Control::ReadInput(void)
{
    arduino.spinOnce();
}

void ControlReadSensorCommand(const std_msgs::String& msg)
{
    if (msg.data == "READ_SENSOR") {
        // Trigger the reading of sensors
        readSensors();   // Read all sensor data
        publishSensorData();  // Publish all sensor data
    }
}

// Function to read all sensor data
void readSensors() {
    ultrasonicDistance = readUltrasonicSensor();
    humidityValue = readHumiditySensor();
    readCompassSensor();
    soilMoistureValue = analogRead(SoilMoisturePin);  // Update with actual code
}

// Function to publish all sensor data
void publishSensorData() {
    std_msgs::Int32 ultrasonic_msg;
    ultrasonic_msg.data = ultrasonicDistance;
    ultrasonic_pub.publish(&ultrasonic_msg);

    std_msgs::Float32 humidity_msg;
    humidity_msg.data = humidityValue;
    humidity_pub.publish(&humidity_msg);

    std_msgs::Float32 compass_msg[3];
    compass_msg[0].data = compassX;
    compass_msg[1].data = compassY;
    compass_msg[2].data = compassZ;
    compass_pub.publish(compass_msg);

    std_msgs::Int32 soil_moisture_msg;
    soil_moisture_msg.data = soilMoistureValue;
    soil_moisture_pub.publish(&soil_moisture_msg);
}
