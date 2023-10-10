#include <ESP32Servo.h>
#include <WiFi.h>
#include <HTTPClient.h>



#define SERVO_1_PIN 12
#define SERVO_2_PIN 13
#define IR_SENSOR_PIN 15
#define LED_PIN 4

// CAMERA_MODEL_AI_THINKER
#include "esp_camera.h"
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22



const char* ssid = "Z60";
const char* password = "12345678";
const char* post_url = "https://trash-detection-frontend.netlify.app/api/hello";


Servo servo_1, servo_2;
int servoPosition_1 = 0, servoPosition_2 = 0;
int increment = 1;
unsigned long previous_timestamp = 0;
unsigned long interval = 15;
int IRSensorValue = LOW;
int servo_1_target = 0;

enum State {
  IDLE,
  CAPTURE,
  MOVE_SERVO_1,
  MOVE_SERVO_2,
  MOVE_SERVO_2_BACK,
  MOVE_SERVO_1_BACK,
};

State currentState = IDLE;
int previousPosition_1 = 0;
int previousPosition_2 = 0;




void captureAndSendPhoto() {
  camera_fb_t* fb = NULL;

  // Capture a photo
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  HTTPClient http;

  Serial.print("[HTTP] begin...\n");
  // configure traged server and url

  http.begin(post_url);  //HTTP

  Serial.print("[HTTP] POST...\n");
  // start connection and send HTTP header
  int httpCode = http.sendRequest("POST", fb->buf, fb->len);  // we simply put the whole image in the post body.

  // httpCode will be negative on error
  if (httpCode > 0) {
    // HTTP header has been send and Server response header has been handled
    Serial.printf("[HTTP] POST... code: %d\n", httpCode);

    // file found at server
    if (httpCode == 200) {
      String payload = http.getString();
      Serial.println(payload);
    }
  } else {
    Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();

  servo_1_target = 180;

  // Return the frame buffer to the camera library
  esp_camera_fb_return(fb);
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

  // PREPARE CAMERA

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }


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

      case CAPTURE:
        digitalWrite(LED_PIN, HIGH);
        currentState = MOVE_SERVO_1;
        break;

      case MOVE_SERVO_1:
        if (servoPosition_1 < 180) {
          servoPosition_1 += increment;
          servo_1.write(servoPosition_1);
        } else {
          currentState = MOVE_SERVO_1_BACK;
        }
        break;

      case MOVE_SERVO_1_BACK:
        if (servoPosition_1 > 0) {
          servoPosition_1 -= increment;
          servo_1.write(servoPosition_1);
        } else {
          currentState = MOVE_SERVO_2;
        }
        break;

      case MOVE_SERVO_2:
        if (servoPosition_2 < 90) {
          servoPosition_2 += increment;
          servo_2.write(servoPosition_2);
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
        }
        break;
      default:
        break;
    }
  }
}



//
