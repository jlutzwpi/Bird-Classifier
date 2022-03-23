#include "esp_camera.h"
#include <WiFi.h>
#include "FS.h"
#include "SD_MMC.h"
#include "time.h"

#include "ESPxWebFlMgr.h"
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#include "camera_pins.h"

const char* ssid = "op";
const char* password = "Granby18!";

const char* ntpServer = "pool.ntp.org";
//-5 from GMT * 60 s/min * 60 min/hr
const long  gmtOffset_sec = -18000;
const int   daylightOffset_sec = 3600;

const word filemanagerport = 8080;
ESPxWebFlMgr filemgr(filemanagerport); 

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define ms_TO_S_FACTOR 1000ULL     /* Conversion factor from milliseconds to seconds */
#define TIME_TO_WAKE_HRS  18       /* Once in deep sleep, will wait 18 hours to wake */
#define s_TO_HOUR_FACTOR 3600      /* 60 seconds in a minute, 60 minutes in a hour */


void startCameraServer();
void setLocalTime(struct tm timeInfo);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  //set up deep sleep -> # hours * uS to s conversion * s to hours conversion
  esp_sleep_enable_timer_wakeup(TIME_TO_WAKE_HRS * uS_TO_S_FACTOR * s_TO_HOUR_FACTOR);
  Serial.println("Setup ESP32 to wake up after " + String(TIME_TO_WAKE_HRS) +  " hours in deep sleep.");

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
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  filemgr.begin();
  
  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  //connect to NTP time server
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  //get the time
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
  }
  else
  {
    setLocalTime(timeinfo);
  }

  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed");
  }
  
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD Card attached");
  }
}
//unsigned long timecheck = 0.0;
void loop() {

  filemgr.handleClient();
  
  //JEL: testing the deep sleep at a certain time
  //plan would be to wakeup during daylight, stay on for a certain period of time (6 hours), then go into deep sleep at night
  //wakeTime is the amount of time that the camera is awake, if this time is exceeded by the run time, go to sleep
  unsigned long wakeTime = 6 * ms_TO_S_FACTOR * s_TO_HOUR_FACTOR;
  if(millis() > wakeTime) {
    Serial.print("Time in ms: ");
    Serial.print(String(millis()));
    Serial.println(". Going to sleep now");
    Serial.flush(); 
    esp_deep_sleep_start();
  }
}
