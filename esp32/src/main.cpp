#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ESP32Servo.h>
#include <MS5837.h>

MS5837 sensor;
Adafruit_ADS1015 ads;

// PWM pins for 6 ESCs
const byte servoPins[6] = {32, 33, 25, 26, 27, 13}; // Change pins according to hardware
Servo thruster[6];        // Array of 6 servo objects

float atmosphericPressure = 0.0;

// Scheduling for various tasks
unsigned long previousDepthTime = 0;
unsigned long previousBatteryTime = 0;
const unsigned long depthInterval = 50;    // Every 50ms
const unsigned long batteryInterval = 5000;  // Every 5 seconds

int direction_A = 0;    // Store direction after parsing numbers
int power_A3 = 00;      // Power A is intended for rear thrusters.
int power_A4 = 00;
int power_A5 = 00;
int direction_B = 0;
int power_B01 = 00;     // Power B is intended for vertical thrusters.
int power_B2 = 00; 
int light = 0;          // 1 ---> on   2 ---> off

const int ledPin = 4; 
//int button = 0;

int counter_error = 0;
bool dataChanged = false;  // Flag to detect changes

void receive_data() {
  char buffer[16]; // 13 byte packet + "\r\n" + '\0'
  
  // Process data
  dataChanged = true; // Add this line
  counter_error = 0; // reset error counter on successful read
  
  // Read until \n
  size_t bytesRead = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);

  if (bytesRead > 0) {
    buffer[bytesRead] = '\0';  // String terminator

    // Remove \r if it's at the end
    if (buffer[bytesRead - 1] == '\r') {
      buffer[bytesRead - 1] = '\0';
    }

    // Now buffer contains only pure data (e.g., "1030456012")
    if (strlen(buffer) == 13) {
      direction_A = buffer[0] - '0';
      power_A3    = (buffer[1] - '0') * 10 + (buffer[2] - '0');
      power_A4    = (buffer[3] - '0') * 10 + (buffer[4] - '0');
      power_A5    = (buffer[5] - '0') * 10 + (buffer[6] - '0');
      direction_B = buffer[7] - '0';
      power_B01   = (buffer[8] - '0') * 10 + (buffer[9] - '0');
      power_B2    = (buffer[10] - '0') * 10 + (buffer[11] - '0');
      light       = buffer[12] - '0';

      //Serial.println("Packet OK");
  
    } else {
      //Serial.println("ERROR: Wrong packet size");
      counter_error++; // Increment error counter
    }
  }
}

// direction == 1 -> Forward    direction == 2 -> backward
void controlThruster(int index, int direction, int power) {
    power = constrain(power, 0, 99);
    int write_thruster;
    if (direction == 1) {
      write_thruster = constrain(map(power, 0, 99, 1525, 1900), 1525, 1900);
      thruster[index].writeMicroseconds(write_thruster);
       
    } else if (direction == 2) {
      write_thruster = constrain(map(power, 0, 99, 1475, 1100), 1100, 1475);
      thruster[index].writeMicroseconds(write_thruster);
       
    } else {
      thruster[index].writeMicroseconds(1500);
    }
}

void check_error() {
  if(counter_error > 20) {
    Serial.println("error!!<< The information received from the gamepad is incorrect. >>!!error");
    Serial.println("error!!<< All engines are turned off. >>!!error");
    for(int i = 0; i < 6; i++) {   // All engines are turned off.
      controlThruster(i, 0, 0); 
    }
  }
}

void printMotorStatus(int motorIndex, int direction, int power) {
  Serial.print("Motor ");
  Serial.print(motorIndex);
  Serial.print(": Dir=");
  Serial.print(direction);
  Serial.print(", Power=");
  Serial.print(power);
  Serial.print(", PWM=");
  Serial.println(thruster[motorIndex].readMicroseconds());
}

void thrusters_A() {
  switch (direction_A){
    case 1:
      controlThruster(5, 1, power_A5);
      controlThruster(4, 1, int(power_A4/4));
      controlThruster(3, 1, int(power_A3/2)); // Divide by 2 for diagonal movement
      //Serial.println("Mode 1: Diagonal movement pattern ");
      break;

    case 2:
      controlThruster(5, 1, power_A5);
      controlThruster(4, 1, power_A4);
      controlThruster(3, 1, power_A3);
      //Serial.println("Mode 2: Forward movement pattern");
      break;

    case 3:
      controlThruster(5, 1, int(power_A5/2));
      controlThruster(4, 1, int(power_A4/4));
      controlThruster(3, 1, power_A3); 
      //Serial.println("Mode 3: Diagonal movement pattern ");
      break;

    case 4:
      controlThruster(5, 2, power_A5);
      controlThruster(4, 0, 0); 
      controlThruster(3, 1, power_A3);
      //Serial.println("Mode 4: Leftward movement pattern ");
      break;

    case 6:
      controlThruster(5, 1, power_A5);
      controlThruster(4, 0, 0); 
      controlThruster(3, 2, power_A3);
      //Serial.println("Mode 6: Rightward movement pattern "); 
      break; 

    case 7:
      controlThruster(5, 2, power_A5);
      controlThruster(4, 2, int(power_A5/4));
      controlThruster(3, 2, int(power_A5/2));
      //Serial.println("Mode 7: Backward diagonal movement pattern "); 
      break;

    case 8:
      controlThruster(5, 2, power_A5);
      controlThruster(4, 2, power_A4);
      controlThruster(3, 2, power_A3);
      //Serial.println("Mode 8: Backward movement pattern "); 
      break;

    case 9:
      controlThruster(5, 2, int(power_A5/2));
      controlThruster(4, 2, int(power_A4/4));
      controlThruster(3, 2, power_A3); 
      //Serial.println("Mode 9: Backward diagonal movement pattern ");
      break;

    default:
      // First turn off all motors
      controlThruster(5, 0, 0);
      controlThruster(4, 0, 0);
      controlThruster(3, 0, 0);
      //Serial.println("Default: All motors stopped");
      break;
  }
  
  //Serial.println("\n--- Thrusters A Status ---");
  //printMotorStatus(5, direction_A, power_A5);
  //printMotorStatus(4, direction_A, power_A4);
  //printMotorStatus(3, direction_A, power_A3);
}

void thrusters_B(){
  switch (direction_B) {
    case 2:
      controlThruster(0, 1, power_B01);
      controlThruster(1, 1, power_B01);
      controlThruster(2, 1, power_B2);
      //Serial.println("Mode 2: Vertical downward"); 
      break;
    case 8:
      controlThruster(0, 2, power_B01);
      controlThruster(1, 2, power_B01);
      controlThruster(2, 2, power_B2); 
      //Serial.println("Mode 8: Vertical upward");
      break;
    case 4:
      controlThruster(0, 1, power_B01);
      controlThruster(1, 1, power_B01);
      //controlThruster(2, 2, power_B2);
      //Serial.println("Mode 2: pitch down"); 
      break;
    case 6:
      controlThruster(0, 2, power_B01);
      controlThruster(1, 2, power_B01);
      //controlThruster(2, 0, power_B2); 
      //Serial.println("Mode 8: pitch up");
      break;
      
    default:
      controlThruster(0, 0, power_B01);
      controlThruster(1, 0, power_B01);
      controlThruster(2, 0, power_B2); 
      //Serial.println("Default: Vertical motors stopped");
      break;
  }

  //Serial.println("\n--- Thrusters B Status ---");
  //printMotorStatus(0, direction_B, power_B01);
  //printMotorStatus(1, direction_B, power_B01);
  //printMotorStatus(2, direction_B, power_B2);
}

void clearSerialBuffer() {
  while (Serial.available() > 0) { // While there is data in the buffer
    Serial.read(); // Read one byte of data and discard it
  }
}

void controlLED(int state) {
  if (state == 1) {
    digitalWrite(ledPin, LOW); // Turn on LED
    Serial.println("LED: ON");
  }
  else if(state == 2) {
    digitalWrite(ledPin, HIGH);  // Turn off LED
    Serial.println("LED: OFF");
  }
}

// LED blinking function during startup
void startupLEDBlink(){
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPin, LOW);
    delay(150); // On for 0.15 seconds
    digitalWrite(ledPin, HIGH);
    delay(150); // Off for 0.15 seconds
  }
}

// ----------------------------------send-depth-----------------------------------
void calibrateAtmosphericPressure() {
  float totalPressure = 0.0;
  int samples = 10;
  for (int i = 0; i < samples; i++) {
    sensor.read();
    totalPressure += sensor.pressure();
    delay(100);
  }
  atmosphericPressure = totalPressure / samples;
}

void send_depth(){
  sensor.read();
  float pressure_mbar = sensor.pressure();
  float depth_m = (pressure_mbar - atmosphericPressure) * 100.0 / (990.0 * 9.81);
  int depth_cm = (int)(depth_m * 100);
  Serial.print("depth:");
  Serial.println(depth_cm); 
}
// --------------------------------------------------------------------------------

// ----------------------------------send-battery-----------------------------------
bool setupBatterySensor() {
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS1015!");
    return false;
  }
  ads.setGain(GAIN_ONE);
  return true;
}

int getBatteryVoltageMillivolt() {
  int16_t adc0 = ads.readADC_SingleEnded(0);
  float voltageSensor = adc0 * 0.002;
  voltageSensor = constrain(voltageSensor, 1.45, 1.6954);
  int batteryVoltage = map(voltageSensor * 1000, 1450, 1695, 16500, 18600);
  return batteryVoltage;
}

void send_battery() {
  int batteryVoltage = getBatteryVoltageMillivolt();
  Serial.print("battery:");
  Serial.println(batteryVoltage);
}
// ---------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50); // Set wait time for receiving data
  Wire.begin();
  
  if (!sensor.init()) {
    Serial.println("Sensor init failed!");
  }
  
  if (!setupBatterySensor()) {
    Serial.println("Battery sensor failed! Stopping...");
  }
  
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(990); // Fresh water density
  calibrateAtmosphericPressure();

  // ESC settings for all motors
  counter_error = 0;
  for (int i = 0; i < 6; i++) {
    thruster[i].setPeriodHertz(50); // 50Hz frequency
    thruster[i].attach(servoPins[i], 1100, 1900); // PWM range
    thruster[i].writeMicroseconds(1500); // Initial neutral value
    delay(500); // Short delay between initializing each ESC
  }
  delay(3000);

  // Set LED pin as output
  pinMode(ledPin, OUTPUT);
  
  // LED blinking during startup
  startupLEDBlink();
}

void loop() {
  receive_data();
  //check_error();
  //if(dataChanged){
  thrusters_A();
  thrusters_B();
  controlLED(light);
  //temperature();
  //pressure();
  
  unsigned long currentTime = millis();

  // send depth every 50ms
  if (currentTime - previousDepthTime >= depthInterval) {
    previousDepthTime = currentTime;
    send_depth();
  }
  
  // send battery every 5 seconds
  if (currentTime - previousBatteryTime >= batteryInterval){
    previousBatteryTime = currentTime; 
    send_battery();
  }
  
  dataChanged = false;
  //}
}