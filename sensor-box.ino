#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

const int THERMISTOR_PIN = A0;
const int NTC_PIN = A1;
const int LDR_PIN = A2;
const int TRIGGER_PIN = 4;
const int ECHO_PIN = 5;
const int VDIVIDER_R1 = 9950;
const int NTC_B_VAL = 4300;
const float T0_KELV = 298.15;

LSM9DS1 imu;

#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

#define PRINT_CALCULATED
#define PRINT_SPEED 250 // 250 ms between prints


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(THERMISTOR_PIN, INPUT);
  pinMode(NTC_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    while (1);
  }
}

unsigned long get_ping() {
  unsigned long duration;
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(TRIGGER_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  return duration;
}

double get_distance_cm() {
  double distance = (get_ping() / 2) / 29.1;
  return distance;
}

double adc_to_resistance(int adc_pin) {
  return (VDIVIDER_R1 / (1023 / (double) analogRead(adc_pin) - 1));
}

double resistance_to_degrees(double r) {
  return NTC_B_VAL / log(r / (VDIVIDER_R1 * exp(-NTC_B_VAL / T0_KELV))) - 273.15;
}

bool check_light() {
  int adc = analogRead(LDR_PIN);
  if (adc > 512) {
    return true;
  }
  else {
    return false;
  }

}

void loop() {

  int distance = get_distance_cm();
  double deg = resistance_to_degrees(adc_to_resistance(NTC_PIN));
  float g[3];
  if ( imu.gyroAvailable() )
  {
    imu.readGyro();

    g[0] = imu.calcGyro(imu.gx);
    g[1] = imu.calcGyro(imu.gy);
    g[2] = imu.calcGyro(imu.gz);

  }
  bool light = check_light();
  
  Serial.print(distance);
  Serial.print(";");
   Serial.print(deg);
  Serial.print(";");
  Serial.print(g[0]);
  Serial.print(";");
  Serial.print(g[1]);
  Serial.print(";");
  Serial.print(g[2]);
  Serial.print(";");
  Serial.print(light);
  Serial.println(";");
  delay(1000);
}

