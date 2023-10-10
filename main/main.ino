#include <ESP32Servo.h>
#include <WiFi.h>
#include <HTTPClient.h>


#define SERVO_1_PIN 12
#define SERVO_2_PIN 13
#define IR_SENSOR_PIN 15
#define LED_PIN 4

Servo servo_1, servo_2;
int servoPosition_1 = 0, servoPosition_2 = 0;
int increment = 1;
unsigned long previous_timestamp = 0;
unsigned long interval = 15;
int IRSensorValue = LOW;

enum State {
  IDLE,
  MOVE_SERVO_1,
  MOVE_SERVO_2,
  TURN_ON_LED,
  MOVE_SERVO_1_BACK,
  MOVE_SERVO_2_BACK
};

State currentState = IDLE;

int previousPosition_1 = 0;
int previousPosition_2 = 0;

const char* ssid = "Z60";
const char* password = "12345678";

void sendGetRequest(String url) {
  // Create an HTTPClient object
  HTTPClient http;

  // Send GET request
  http.begin(url);

  // Start the GET request
  int httpResponseCode = http.GET();

  // Check for a successful request
  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);

    String payload = http.getString();
    Serial.println("Response:");
    Serial.println(payload);
  } else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }

  // Close the connection
  http.end();
}

void setup() {
  // begin
  Serial.begin(9600);


  // connecting to WIFI
  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

  // attach servos and sensors to appropriate pins
  servo_1.attach(SERVO_1_PIN);
  servo_2.attach(SERVO_2_PIN);
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // initialize the servo's to zero
  servo_1.write(0);
  servo_2.write(0);
  delay(1000);

  // Initialize previous positions
  previousPosition_1 = 0;
  previousPosition_2 = 0;

  // Send a GET request and handle the response
  // sendGetRequest("https://akuzechie.blogspot.com/2021/09/assembly-programming-via-arduino-uno.html");

}



void loop() {

  IRSensorValue = !digitalRead(IR_SENSOR_PIN);

  if (IRSensorValue == HIGH && currentState == IDLE) {
    currentState = MOVE_SERVO_1;
  }

  unsigned long current_timestamp = millis();
  if (current_timestamp - previous_timestamp >= interval) {
    previous_timestamp = current_timestamp;

    switch (currentState) {
      case MOVE_SERVO_1:
        if (servoPosition_1 < 90) {
          servoPosition_1 += increment;
          servo_1.write(servoPosition_1);
        } else {
          currentState = MOVE_SERVO_2;
        }
        break;
      case MOVE_SERVO_2:
        if (servoPosition_2 < 180) {
          servoPosition_2 += increment;
          servo_2.write(servoPosition_2);
        } else {
          currentState = TURN_ON_LED;
        }
        break;
      case TURN_ON_LED:
        digitalWrite(LED_PIN, HIGH);
        currentState = MOVE_SERVO_1_BACK;
        break;
      case MOVE_SERVO_1_BACK:
        if (servoPosition_1 > 0) {
          servoPosition_1 -= increment;
          servo_1.write(servoPosition_1);
        } else {
          currentState = MOVE_SERVO_2_BACK;
        }
        break;
      case MOVE_SERVO_2_BACK:
        if (servoPosition_2 > 0) {
          servoPosition_2 -= increment;
          servo_2.write(servoPosition_2);
        } else {
          currentState = IDLE;
          digitalWrite(LED_PIN, LOW);  // Turn off LED
        }
        break;
      default:
        break;
    }
  }
}