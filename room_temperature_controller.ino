#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

#define DHTPIN 2       // Digital pin connected to the DHT11 sensor
#define DHTTYPE DHT11  // DHT11 sensor type
DHT dht(DHTPIN, DHTTYPE);

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Initialize the LCD with I2C address 0x27, 16 columns, and 2 rows

const int fanPin1 = 3;  // Digital pin connected to IN1 of the motor driver
const int fanPin2 = 4;  // Digital pin connected to IN2 of the motor driver
const int motorSpeedPin = 9;  // PWM pin connected to ENA of the motor driver

void setup() {
  Serial.begin(9600);  // Initialize serial communication for debugging
  lcd.init();         // Initialize the LCD
  lcd.backlight();    // Turn on the backlight (if available)
  dht.begin();        // Initialize the DHT11 sensor

  lcd.setCursor(0, 0);     // Set cursor to the first row, first column
  lcd.print("Temp (C): "); // Display "Temp (C): " on LCD

  pinMode(fanPin1, OUTPUT);       // Set fan control pins as output
  pinMode(fanPin2, OUTPUT);
  pinMode(motorSpeedPin, OUTPUT); // Set motor speed control pin as output
}

void loop() {
  float temperature = dht.readTemperature();  // Read temperature from DHT11 sensor in Celsius
  Serial.print("Temperature: ");
  Serial.println(temperature);

  lcd.setCursor(10, 0);    // Set cursor to the first row, eleventh column (after "Temp (C): ")
  lcd.print(temperature);  // Display temperature on LCD

  if (temperature > 36) {
    Serial.println("Turning fan on");
    analogWrite(motorSpeedPin, 255);  // Set motor speed to maximum (255) when turning the fan on
    digitalWrite(fanPin1, HIGH);      // Set IN1 to HIGH
    digitalWrite(fanPin2, LOW);       // Set IN2 to LOW
  } else {
    Serial.println("Turning fan off");
    analogWrite(motorSpeedPin, 0);    // Set motor speed to 0 when turning the fan off
    digitalWrite(fanPin1, LOW);       // Set IN1 to LOW
    digitalWrite(fanPin2, LOW);       // Set IN2 to LOW
  }

  delay(2000);  // Delay for 2 seconds before updating readings and fan control
}