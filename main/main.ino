#include <ESP32Servo.h>  // Servo Library to move a 180 deg servo precisely with ESP32_cam
#include <WiFi.h>        // wifi library to connect to a WIFI network
#include <HTTPClient.h>  // http client library to send http request


// These are the GPIO pins used for the project
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
const char* post_url = "https://trash-detection-frontend.netlify.app/api/image-bytes-to-data-url";          // this endpoint takes in the raw image data and returns a data url
const char* prediction_server_url = "https://trash-detection-api1.onrender.com/predict";  // this endpoint takes in a data url and returns the prediction using AI


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

// state analysis:
// when the IR sensor detects an object, capture image and send to server -> move servo 1 to a position -> move servo 2 downwards -> then upwords -> then servo 1 to idle position

State currentState = IDLE;  // initial state
int previousPosition_1 = 0;
int previousPosition_2 = 0;




void captureAndSendPhoto() {
  camera_fb_t* fb = NULL;  // holds the frame buffer (raw image data)

  digitalWrite(LED_PIN, HIGH);  // flash for better image quality
  // Capture a photo
  fb = esp_camera_fb_get();  // capture the image
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  digitalWrite(LED_PIN, LOW);

  HTTPClient http;

  Serial.println("[HTTP] begin...");
  // configure traged server and url

  http.begin(post_url);  //HTTP

  Serial.println("[HTTP] POST...");
  // start connection and send HTTP header
  int httpCode = http.sendRequest("POST", fb->buf, fb->len);  // we simply put the whole image in the post body.

  // httpCode will be negative on error
  if (httpCode > 0) {
    // HTTP header has been send and Server response header has been handled
    Serial.printf("[HTTP] POST... code: %d\n", httpCode);

    // file found at server
    if (httpCode == 200) {
      String payload = http.getString();
      // Serial.println(payload);
    }
  } else {
    Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();

  servo_1_target = 180;  // later on will be changed by the payload to different angles

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
    // not yet connected
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

  // PREPARE CAMERA
  // camera config
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

  // camera initialization
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
}



void loop() {

  // take the infra-red sensor value
  IRSensorValue = !digitalRead(IR_SENSOR_PIN);

  // if the sensor detects something and is in the IDLE state
  if (IRSensorValue == HIGH && currentState == IDLE) {
    // go to capture state
    currentState = CAPTURE;
  }

  // interval based structure - non blocking (instead of delay based)

  unsigned long current_timestamp = millis();

  // if it has been long enough (based on interval), do a work cycle
  if (current_timestamp - previous_timestamp >= interval) {

    previous_timestamp = current_timestamp;

    // do different action based on the state
    switch (currentState) {

      case CAPTURE:
        Serial.println("Capturing Image");
        captureAndSendPhoto();

        // move servo 1 -> to target (target updated through the prev function)
        currentState = MOVE_SERVO_1;
        Serial.println("Moving servo 1 to target");

        break;

      case MOVE_SERVO_1:
        if (servoPosition_1 < servo_1_target) {
          servoPosition_1 += increment;
          servo_1.write(servoPosition_1);
        } else {

          // tilt servo 2 90 degree (downward)
          currentState = MOVE_SERVO_2;
          Serial.println("Moving servo 2 to target");
        }
        break;

      case MOVE_SERVO_2:

        if (servoPosition_2 < 90) {
          servoPosition_2 += increment;
          servo_2.write(servoPosition_2);
        } else {
          // tilt back servo 2 - 0 degree (upword)
          currentState = MOVE_SERVO_2_BACK;
          Serial.println("Moving servo 2 back");
        }
        break;

      case MOVE_SERVO_2_BACK:

        if (servoPosition_2 > 0) {
          servoPosition_2 -= increment;
          servo_2.write(servoPosition_2);
        } else {
          // servo 1 to default position
          currentState = MOVE_SERVO_1_BACK;
          Serial.println("Moving servo 1 back");
        }
        break;

      case MOVE_SERVO_1_BACK:

        if (servoPosition_1 > 0) {
          servoPosition_1 -= increment;
          servo_1.write(servoPosition_1);
        } else {
          currentState = IDLE;
          Serial.println("Going Idle");
        }
        break;


      default:
        break;
    }
  }
}
